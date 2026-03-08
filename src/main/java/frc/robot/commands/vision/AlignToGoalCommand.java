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
 * Vision-based alignment to a goal that is offset from an AprilTag.
 * 
 * Uses an AprilTag as a reference point and aligns the robot to a goal position
 * that is offset from the tag. Useful for aligning to goals that have a known
 * offset relative to the AprilTag reference frame.
 * 
 * Each axis is controlled independently:
 * - X (Forward/Backward): Distance to goal (target: goalDistanceX)
 * - Y (Left/Right): Lateral offset to goal (target: goalOffsetY)
 * - Yaw (Rotation): Facing the goal (target: goalYaw)
 * 
 * All measurements are in TAG frame, then adjusted by goal offset.
 */
public class AlignToGoalCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_targetId;

  // Independent PID controllers for each axis
  private final PIDController m_xController;    // Distance control
  private final PIDController m_yController;    // Lateral control
  private final PIDController m_yawController;  // Rotation control

  // Goal offset from tag (in tag frame)
  private final double m_goalOffsetX;   // Forward/back offset from tag (meters)
  private final double m_goalOffsetY;   // Left/right offset from tag (meters)
  private final double m_goalYaw;       // Heading at goal (degrees, 0° = facing tag)

  // Target values relative to goal
  private static final double TARGET_DISTANCE_FROM_GOAL = 0.0;  // Reach the goal exactly
  private static final double TARGET_LATERAL_AT_GOAL = 0.0;     // Centered at goal
  private static final double TARGET_YAW_AT_GOAL = 0.0;          // Use specified goal yaw

  /**
   * Creates a new AlignToGoalCommand.
   *
   * @param drive the drive subsystem
   * @param vision the vision subsystem
   * @param targetId the AprilTag ID to use as reference
   * @param goalOffsetX forward/back offset from tag to goal (meters)
   * @param goalOffsetY left/right offset from tag to goal (meters)
   * @param goalYaw desired heading at goal (degrees, 0° = facing tag direction)
   */
  public AlignToGoalCommand(DriveSubsystem drive, VisionSubsystem vision, int targetId,
      double goalOffsetX, double goalOffsetY, double goalYaw) {
    m_drive = drive;
    m_vision = vision;
    m_targetId = targetId;
    m_goalOffsetX = goalOffsetX;
    m_goalOffsetY = goalOffsetY;
    m_goalYaw = goalYaw;

    // X Controller: Distance to goal
    m_xController = new PIDController(
        VisionAlignmentConstants.kVisionXP,
        VisionAlignmentConstants.kVisionXI,
        VisionAlignmentConstants.kVisionXD);
    m_xController.setTolerance(VisionAlignmentConstants.kXTolerance);

    // Y Controller: Lateral offset at goal
    m_yController = new PIDController(
        VisionAlignmentConstants.kVisionYP,
        VisionAlignmentConstants.kVisionYI,
        VisionAlignmentConstants.kVisionYD);
    m_yController.setTolerance(VisionAlignmentConstants.kYTolerance);

    // Yaw Controller: Rotation to goal heading
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
    System.out.println("AlignToGoalCommand: Targeting goal offset (" + m_goalOffsetX + "m, " + m_goalOffsetY + "m) from tag ID " + m_targetId);
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
    double lateralOffsetToTag = cameraLateralOffset + cameraOffsetY;
    
    // Calculate the angular offset caused by camera position
    double angularOffsetRad = Math.atan2(cameraOffsetY, cameraDistanceX);
    double angularOffsetDeg = Math.toDegrees(angularOffsetRad);
    
    // Correct the yaw: add the angular offset to get robot-to-target yaw
    double robotYawToTag = cameraYaw + angularOffsetDeg;

    // ========== GOAL CALCULATION: Apply goal offset to get goal position ==========
    // The goal is offset from the tag by (goalOffsetX, goalOffsetY) in tag frame
    // We need to calculate robot-to-goal measurements
    
    // Distance from robot to goal
    // tag is at (distanceToTag, lateralOffsetToTag) from robot
    // goal is at (goalOffsetX, goalOffsetY) from tag
    // so goal is at (distanceToTag - goalOffsetX, lateralOffsetToTag - goalOffsetY) from robot
    double distanceToGoal = distanceToTag - m_goalOffsetX;
    double lateralOffsetToGoal = lateralOffsetToTag - m_goalOffsetY;
    
    // Heading to goal - calculate angular offset from robot center to goal
    // Similar to camera offset math: atan2(lateral_offset, forward_distance)
    // This gives us the angle from robot center to the goal location
    double goalAngularOffsetRad = Math.atan2(lateralOffsetToGoal, distanceToGoal);
    double goalAngularOffsetDeg = Math.toDegrees(goalAngularOffsetRad);
    
    // Robot's heading should point at the goal location
    // This is the tag direction plus the offset to the goal
    double robotYawToGoal = robotYawToTag + goalAngularOffsetDeg;
    
    // Apply desired heading offset at goal (if you want robot to face different direction than goal)
    robotYawToGoal = robotYawToGoal + m_goalYaw;
    
    // Normalize to [-180, 180]
    while (robotYawToGoal > 180) robotYawToGoal -= 360;
    while (robotYawToGoal < -180) robotYawToGoal += 360;

    // ========== CONTROL: Independent PID for each axis ==========
    // X-axis: Control distance (move toward/away from goal)
    double xSpeed = -m_xController.calculate(distanceToGoal, TARGET_DISTANCE_FROM_GOAL);

    // Y-axis: Control lateral position (strafe left/right to center at goal)
    double ySpeed = m_yController.calculate(lateralOffsetToGoal, TARGET_LATERAL_AT_GOAL);
    
    // Distance-based adaptive yaw tolerance: expand when close to goal
    // because angle measurement becomes unstable near goal (small distance = large angle variation)
    double baseYawTolerance = VisionAlignmentConstants.kYawTolerance;
    if (distanceToGoal < 0.5) {
      double distanceFactor = Math.max(1.0, 2.0 * (0.5 - distanceToGoal) / 0.5);
      m_yawController.setTolerance(baseYawTolerance * distanceFactor);
    } else {
      m_yawController.setTolerance(baseYawTolerance);
    }
    
    // Yaw-axis: Control rotation (face the goal direction)
    double rotSpeed = m_yawController.calculate(robotYawToGoal, TARGET_YAW_AT_GOAL);

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
    // Show goal position measurements
    SmartDashboard.putNumber("Vision/Goal_Distance_X", distanceToGoal);
    SmartDashboard.putNumber("Vision/Goal_Offset_Y", lateralOffsetToGoal);
    SmartDashboard.putNumber("Vision/Goal_Yaw", robotYawToGoal);
    
    // Also show tag measurements for reference
    SmartDashboard.putNumber("Vision/Tag_Distance_X", distanceToTag);
    SmartDashboard.putNumber("Vision/Tag_Offset_Y", lateralOffsetToTag);
    SmartDashboard.putNumber("Vision/Tag_Yaw", robotYawToTag);
    
    // Debug telemetry
    SmartDashboard.putNumber("Vision/Goal_Angular_Offset", goalAngularOffsetDeg);
    SmartDashboard.putString("Vision/Status", "Goal offset from Tag " + t.getFiducialId());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    SmartDashboard.putString("Vision/Status", interrupted ? "Goal Align Interrupted" : "Goal Align Complete");
  }

  @Override
  public boolean isFinished() {
    // Never finish - continuously hold position while button is held
    return false;
  }
}
