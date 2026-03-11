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
 * Vision-based alignment to shoot from a desired offset relative to a goal.
 * 
 * Similar to CoordinatedHeadingCommand but aligns to a virtual "goal" that is offset
 * from an AprilTag. Useful for basketball-style games where you want to score from
 * specific positions relative to the goal.
 * 
 * The command uses two sets of offsets:
 * 1. Tag-to-Goal offset: Defines where the goal is relative to the AprilTag
 *    (e.g., hoop is 0.5m forward and 0.1m left of the tag)
 * 2. Desired Robot-to-Goal offset: Where the robot should position itself relative to the goal
 *    (e.g., robot should be 1.5m away and centered to shoot)
 * 
 * Each axis is controlled independently:
 * - X (Forward/Backward): Robot-to-goal distance (target: desiredRobotOffsetX)
 * - Y (Left/Right): Robot-to-goal lateral offset (target: desiredRobotOffsetY)
 * - Yaw (Rotation): Robot heading to face goal (target: specified yaw)
 * 
 * Chain of transformations:
 * Camera -> Robot (accounting for camera mounting)
 * -> Tag (via vision measurements)
 * -> Goal (apply tag-to-goal offsets)
 * -> Robot desired position (align to desired robot-to-goal offsets)
 */
public class AlignToGoalCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_targetId;

  // Independent PID controllers for each axis
  private final PIDController m_xController;    // Distance control
  private final PIDController m_yController;    // Lateral control
  private final PIDController m_yawController;  // Rotation control

  // Tag-to-Goal offset (defines where the goal is relative to the tag, in tag frame)
  private final double m_tagToGoalOffsetX;   // Forward/back offset from tag to goal (meters)
  private final double m_tagToGoalOffsetY;   // Left/right offset from tag to goal (meters)

  // Desired Robot-to-Goal offset (defines where robot should be relative to the goal)
  private final double m_desiredRobotOffsetX;  // Forward/back distance from goal (meters)
  private final double m_desiredRobotOffsetY;  // Left/right distance from goal (meters)
  private final double m_desiredYaw;           // Heading at desired position (degrees, 0° = facing tag)

  /**
   * Creates a new AlignToGoalCommand for positioned shooting.
   *
   * @param drive the drive subsystem
   * @param vision the vision subsystem
   * @param targetId the AprilTag ID to use as reference
   * @param tagToGoalOffsetX forward/back offset from tag to goal (meters)
   * @param tagToGoalOffsetY left/right offset from tag to goal (meters)
   * @param desiredRobotOffsetX desired forward/back distance from goal (meters, positive = away from goal)
   * @param desiredRobotOffsetY desired left/right distance from goal (meters)
   * @param desiredYaw desired heading at shooting position (degrees, 0° = facing tag direction)
   */
  public AlignToGoalCommand(DriveSubsystem drive, VisionSubsystem vision, int targetId,
      double tagToGoalOffsetX, double tagToGoalOffsetY,
      double desiredRobotOffsetX, double desiredRobotOffsetY, double desiredYaw) {
    m_drive = drive;
    m_vision = vision;
    m_targetId = targetId;
    m_tagToGoalOffsetX = tagToGoalOffsetX;
    m_tagToGoalOffsetY = tagToGoalOffsetY;
    m_desiredRobotOffsetX = desiredRobotOffsetX;
    m_desiredRobotOffsetY = desiredRobotOffsetY;
    m_desiredYaw = desiredYaw;

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
    // NOTE: Using 5.0° tolerance to account for camera mounting offset on devbot
    m_yawController = new PIDController(
        VisionAlignmentConstants.kVisionYawP,
        VisionAlignmentConstants.kVisionYawI,
        VisionAlignmentConstants.kVisionYawD);
    m_yawController.enableContinuousInput(-180, 180);
    m_yawController.setTolerance(5.0);  // degrees - increased from kYawTolerance

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    System.out.println("AlignToGoalCommand: Aligning to goal at offset (" + m_tagToGoalOffsetX + "m, " + m_tagToGoalOffsetY + "m) from tag ID " + m_targetId + 
                       ". Desired robot position: (" + m_desiredRobotOffsetX + "m, " + m_desiredRobotOffsetY + "m) from goal.");
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

    // ========== GOAL CALCULATION: Apply tag-to-goal offset ==========
    // The goal is offset from the tag by (tagToGoalOffsetX, tagToGoalOffsetY) in tag frame
    // Goal position in robot frame: (distanceToTag + tagToGoalOffsetX, lateralOffsetToTag + tagToGoalOffsetY)
    // But we think in terms of robot-to-goal, so flip the sign
    double goalX = distanceToTag - m_tagToGoalOffsetX;
    double goalY = lateralOffsetToTag - m_tagToGoalOffsetY;
    
    // ========== DESIRED POSITION CALCULATION: Calculate error from desired robot offset ==========
    // We want the robot to be at position (desiredRobotOffsetX, desiredRobotOffsetY) from the goal
    // Current robot position relative to goal is (goalX, goalY)
    // Error is how far we are from the desired position
    double distanceError = goalX - m_desiredRobotOffsetX;   // Positive = too close, negative = too far
    double lateralError = goalY - m_desiredRobotOffsetY;    // Positive = too far right, negative = too far left
    
    // Heading calculation: Calculate robot's desired yaw from goal direction
    // The goal is at position (goalX, goalY) from robot
    // Calculate the angle to the goal using atan2, same as AlignToTagCommand pattern
    double goalAngularOffsetRad = Math.atan2(goalY, goalX);
    double goalAngularOffsetDeg = Math.toDegrees(goalAngularOffsetRad);
    
    // Desired heading to goal: robot's yaw to tag + offset to goal + specified yaw offset
    // This is the heading the robot should have to face the goal
    double desiredHeading = robotYawToTag + goalAngularOffsetDeg + m_desiredYaw;
    
    // Normalize to [-180, 180]
    while (desiredHeading > 180) desiredHeading -= 360;
    while (desiredHeading < -180) desiredHeading += 360;

    // ========== CONTROL: Independent PID for each axis ==========
    // X-axis: Control distance (move toward/away to reach desired offset distance)
    // NEGATED - matches AlignToTagCommand
    double xSpeed = -m_xController.calculate(distanceError, 0.0);

    // Y-axis: Control lateral position (strafe to centered position at desired range)
    // NO negation - matches AlignToTagCommand
    double ySpeed = m_yController.calculate(lateralError, 0.0);
    
    // Distance-based adaptive yaw tolerance: expand when close to desired position
    // because angle measurement becomes unstable at close distances
    // NOTE: Using 5.0° base tolerance to account for camera mounting offset on devbot
    // TODO: Calibrate camera position offsets and reduce back to kYawTolerance (2.0°)
    double baseYawTolerance = 5.0;  // degrees - increased from VisionAlignmentConstants.kYawTolerance
    if (distanceError < 0.5) {
      double distanceFactor = Math.max(1.0, 2.0 * (0.5 - Math.abs(distanceError)) / 0.5);
      m_yawController.setTolerance(baseYawTolerance * distanceFactor);
    } else {
      m_yawController.setTolerance(baseYawTolerance);
    }
    
    // Yaw-axis: Control rotation (face the desired heading)
    // NO negation - matches AlignToTagCommand
    double rotSpeed = m_yawController.calculate(desiredHeading, 0.0);

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
    m_drive.drive(xSpeed, ySpeed, 0.5*rotSpeed, false);

    // ========== TELEMETRY: Dashboard ==========
    // Show desired robot position measurements
    SmartDashboard.putNumber("Vision/Distance_Error", distanceError);
    SmartDashboard.putNumber("Vision/Lateral_Error", lateralError);
    SmartDashboard.putNumber("Vision/Desired_Heading", desiredHeading);
    SmartDashboard.putNumber("Vision/Current_Heading", m_drive.getHeading());
    
    // Also show goal position for reference
    SmartDashboard.putNumber("Vision/Goal_X", goalX);
    SmartDashboard.putNumber("Vision/Goal_Y", goalY);
    SmartDashboard.putNumber("Vision/Goal_Angular_Offset", goalAngularOffsetDeg);
    
    // Show tag measurements for reference
    SmartDashboard.putNumber("Vision/Tag_Distance_X", distanceToTag);
    SmartDashboard.putNumber("Vision/Tag_Offset_Y", lateralOffsetToTag);
    SmartDashboard.putNumber("Vision/Tag_Yaw", robotYawToTag);
    
    SmartDashboard.putString("Vision/Status", "Aligning to goal from Tag " + t.getFiducialId());
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
