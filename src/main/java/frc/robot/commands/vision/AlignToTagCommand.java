// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionAlignmentConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Simple vision-based alignment command using 3 independent PID controllers.
 * 
 * Each axis is controlled independently:
 * - X (Forward/Backward): Distance to tag (target: 1.5m)
 * - Y (Left/Right): Lateral offset using yaw angle (target: centered)
 * - Yaw (Rotation): Facing the tag (target: 0° yaw)
 * 
 * All measurements are in TAG frame for simplicity.
 */
public class AlignToTagCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_targetId;

  // Independent PID controllers for each axis
  private final PIDController m_xController;    // Distance control
  private final PIDController m_yController;    // Lateral control
  private final PIDController m_yawController;  // Rotation control

  // Target values (in tag frame)
  private static final double TARGET_DISTANCE_X = 1.5;  // meters
  private static final double TARGET_OFFSET_Y = 0.0;    // meters (centered)
  private static final double TARGET_YAW = 0.0;         // degrees (facing tag)

  /**
   * Creates a new AlignToTagCommand.
   *
   * @param drive the drive subsystem
   * @param vision the vision subsystem
   * @param targetId the AprilTag ID to align to (-1 for any tag)
   */
  public AlignToTagCommand(DriveSubsystem drive, VisionSubsystem vision, int targetId) {
    m_drive = drive;
    m_vision = vision;
    m_targetId = targetId;

    // X Controller: Distance to tag
    m_xController = new PIDController(
        VisionAlignmentConstants.kVisionXP,
        VisionAlignmentConstants.kVisionXI,
        VisionAlignmentConstants.kVisionXD);
    m_xController.setTolerance(VisionAlignmentConstants.kXTolerance);

    // Y Controller: Lateral offset (yaw-based)
    m_yController = new PIDController(
        VisionAlignmentConstants.kVisionYP,
        VisionAlignmentConstants.kVisionYI,
        VisionAlignmentConstants.kVisionYD);
    m_yController.setTolerance(VisionAlignmentConstants.kYTolerance);

    // Yaw Controller: Rotation to face tag
    m_yawController = new PIDController(
        VisionAlignmentConstants.kVisionYawP,
        VisionAlignmentConstants.kVisionYawI,
        VisionAlignmentConstants.kVisionYawD);
    m_yawController.enableContinuousInput(-180, 180);
    m_yawController.setTolerance(VisionAlignmentConstants.kYawTolerance);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    System.out.println("AlignToTagCommand: Targeting tag ID " + m_targetId);
    m_xController.reset();
    m_yController.reset();
    m_yawController.reset();
  }

  @Override
  public void execute() {
    // Search for the specific target ID from either camera
    var targetFromFrontLeft = m_vision.getFrontLeftTargetById(m_targetId);
    var targetFromFrontRight = m_vision.getFrontRightTargetById(m_targetId);

    if (!targetFromFrontLeft.isPresent() && !targetFromFrontRight.isPresent()) {
      m_drive.drive(0, 0, 0, false);
      SmartDashboard.putString("Vision/Status", "Target " + m_targetId + " not found");
      return;
    }

    // Prefer front left camera if both have the target, otherwise use whichever has it
    boolean useFrontLeft = targetFromFrontLeft.isPresent();
    var target = useFrontLeft ? targetFromFrontLeft : targetFromFrontRight;

    if (target.isEmpty()) {
      m_drive.drive(0, 0, 0, false);
      return;
    }

    var t = target.get();

    // ========== MEASUREMENT: Get tag measurements ==========
    double cameraYaw = t.getYaw();       // Horizontal angle error from camera (degrees)

    // Get camera-to-target transformation which includes distance
    var transform = t.getBestCameraToTarget();
    double cameraDistanceX = transform.getX();  // Forward distance in meters from camera
    
    // Account for camera offset from robot center
    // FrontLeft camera is at (wheelbase/2, -trackwidth/2)
    // FrontRight camera is at (wheelbase/2, +trackwidth/2)
    double cameraOffsetX = DriveConstants.kWheelBase / 2;
    double cameraOffsetY = useFrontLeft ? (-DriveConstants.kTrackWidth / 2) : (DriveConstants.kTrackWidth / 2);
    
    // Transform from camera frame to robot center frame
    // The robot-to-tag distance is the camera distance plus the forward offset
    double distanceToTag = cameraDistanceX - cameraOffsetX;
    
    // Estimate lateral offset from camera yaw
    double cameraLateralOffset = cameraDistanceX * Math.tan(Math.toRadians(cameraYaw));
    
    // Adjust for camera Y position offset to get robot-to-target lateral offset
    // The error is how far the robot is from being centered on the tag
    // Note: We ADD the offset because we need to correct for where the camera is mounted
    double lateralOffset = cameraLateralOffset + cameraOffsetY;
    
    // Calculate the angular offset caused by camera position
    // This is the angle from robot center to camera center
    double angularOffsetRad = Math.atan2(cameraOffsetY, cameraDistanceX);
    double angularOffsetDeg = Math.toDegrees(angularOffsetRad);
    
    // Correct the yaw: add the angular offset to get robot-to-target yaw
    // (We add because the offset correction works the same way as the lateral offset)
    double robotYaw = cameraYaw + angularOffsetDeg;

    // ========== CONTROL: Independent PID for each axis ==========
    // X-axis: Control distance (move toward/away from tag)
    double xSpeed = -m_xController.calculate(distanceToTag, TARGET_DISTANCE_X);

    // Y-axis: Control lateral position (strafe left/right to center)
    double ySpeed = m_yController.calculate(lateralOffset, TARGET_OFFSET_Y);
    
    // Yaw-axis: Control rotation (face the tag) using robot-to-target yaw
    double rotSpeed = m_yawController.calculate(robotYaw, TARGET_YAW);

    // ========== LIMIT: Clamp outputs ==========
    xSpeed = MathUtil.clamp(xSpeed, -0.5, 0.5);
    ySpeed = MathUtil.clamp(ySpeed, -0.5, 0.5);
    rotSpeed = MathUtil.clamp(rotSpeed, -0.5, 0.5);

    // ========== HOLD: Stop movement when at setpoint to eliminate jitter ==========
    if (m_xController.atSetpoint()) {
      xSpeed = 0;
    }
    if (m_yController.atSetpoint()) {
      ySpeed = 0;
    }
    if (m_yawController.atSetpoint()) {
      rotSpeed = 0;
    }

    // ========== COMMAND: Send to robot ==========
    // Using robot-relative coordinates (false = not field relative)
    m_drive.drive(xSpeed, ySpeed, rotSpeed, false);

    // ========== TELEMETRY: Dashboard ==========
    // Show only essential robot-to-tag measurements
    SmartDashboard.putNumber("Vision/Robot_X", distanceToTag);
    SmartDashboard.putNumber("Vision/Robot_Y", lateralOffset);
    SmartDashboard.putNumber("Vision/Robot_Yaw", robotYaw);
    
    // Debug telemetry to help tune camera offset
    SmartDashboard.putNumber("Vision/Camera_Y_Offset", cameraOffsetY);
    SmartDashboard.putNumber("Vision/Camera_Lateral_Offset", cameraLateralOffset);
    SmartDashboard.putNumber("Vision/Camera_Yaw", cameraYaw);
    SmartDashboard.putString("Vision/Status", "Tag " + t.getFiducialId());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    SmartDashboard.putString("Vision/Status", interrupted ? "Interrupted" : "Complete");
  }

  @Override
  public boolean isFinished() {
    // Never finish - continuously hold position while button is held
    return false;
  }
}
