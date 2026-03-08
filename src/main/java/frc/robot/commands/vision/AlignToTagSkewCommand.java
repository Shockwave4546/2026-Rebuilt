package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionAlignmentConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Vision alignment command that aligns the robot to be perpendicular to the tag.
 * Uses the tag's Z angle (theta/rotation) to determine if robot is square/perpendicular to the tag.
 * Target: Z angle = 180° (robot perpendicular to tag plane)
 */
public class AlignToTagSkewCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_targetId;

  // Yaw controller for rotation
  private final PIDController m_yawController;

  // Target Z angle (180 degrees = perpendicular to tag)
  private static final double TARGET_Z_ANGLE = 180.0;

  public AlignToTagSkewCommand(DriveSubsystem drive, VisionSubsystem vision, int targetId) {
    m_drive = drive;
    m_vision = vision;
    m_targetId = targetId;

    // Initialize Yaw controller
    m_yawController = new PIDController(
        VisionAlignmentConstants.kVisionYawP,
        VisionAlignmentConstants.kVisionYawI,
        VisionAlignmentConstants.kVisionYawD);
    m_yawController.setTolerance(VisionAlignmentConstants.kYawTolerance);
    m_yawController.enableContinuousInput(-180, 180);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    System.out.println("AlignToTagSkewCommand: Aligning robot perpendicular to tag (Z angle = 180°) for ID " + m_targetId);
    m_yawController.reset();
  }

  @Override
  public void execute() {
    var targetFromFrontLeft = m_vision.getFrontLeftTargetById(m_targetId);
    var targetFromFrontRight = m_vision.getFrontRightTargetById(m_targetId);

    if (!targetFromFrontLeft.isPresent() && !targetFromFrontRight.isPresent()) {
      m_drive.drive(0, 0, 0, false);
      SmartDashboard.putString("Vision/Status", "Target " + m_targetId + " not found");
      return;
    }

    var target = targetFromFrontLeft.isPresent() ? targetFromFrontLeft : targetFromFrontRight;
    if (target.isEmpty()) {
      m_drive.drive(0, 0, 0, false);
      return;
    }

    var t = target.get();

    // Get the tag's Z angle (theta) - rotation around the camera's Z axis
    // This tells us if the robot is perpendicular to the tag
    // 180° = perpendicular to tag plane (robot facing tag)
    // 0° = parallel to tag plane (robot beside tag)
    double zAngle = t.getBestCameraToTarget().getRotation().getZ();
    double zAngleDegrees = Math.toDegrees(zAngle);
    
    // Get distance to tag for adaptive control
    double cameraDistanceX = t.getBestCameraToTarget().getX();
    
    // The error is how far we are from 180 degrees (perpendicular)
    // We need to normalize to handle the wraparound at ±180
    double angleError = zAngleDegrees - TARGET_Z_ANGLE;
    
    // Normalize error to [-180, 180]
    while (angleError > 180) angleError -= 360;
    while (angleError < -180) angleError += 360;

    // Distance-based adaptive deadzone: expand tolerance when far away
    // because Z-angle measurement becomes less stable at distance
    // At 1.5m: use base tolerance, linearly expand to 3x at 2.5m+
    double baseTolerance = VisionAlignmentConstants.kYawTolerance;
    if (cameraDistanceX > 1.5) {
      double distanceFactor = Math.min(3.0, 1.0 + (cameraDistanceX - 1.5) / 0.5);
      m_yawController.setTolerance(baseTolerance * distanceFactor);
    } else {
      m_yawController.setTolerance(baseTolerance);
    }

    // Yaw control: rotate to be perpendicular to tag
    double rotSpeed = -m_yawController.calculate(angleError, 0);
    rotSpeed = MathUtil.clamp(rotSpeed, -0.5, 0.5);

    // Stop movement when at setpoint to eliminate jitter
    if (m_yawController.atSetpoint()) {
      rotSpeed = 0;
    }

    // Only rotate, no X or Y movement
    m_drive.drive(0, 0, rotSpeed, false);

    // Telemetry
    SmartDashboard.putNumber("Vision/Tag_Z_Angle", zAngleDegrees);
    SmartDashboard.putNumber("Vision/Z_Angle_Error", angleError);
    SmartDashboard.putNumber("Vision/Rot_Speed", rotSpeed);
    SmartDashboard.putNumber("Vision/Camera_Distance_X", cameraDistanceX);
    SmartDashboard.putString("Vision/Status", "Z-Angle Align - Tag " + t.getFiducialId());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    SmartDashboard.putString("Vision/Status", interrupted ? "Z-Angle Interrupted" : "Z-Angle Complete");
  }

  @Override
  public boolean isFinished() {
    // Never finish - continuously hold position while button is held
    return false;
  }
}
