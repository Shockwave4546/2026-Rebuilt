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
 * Vision alignment command that controls only the Y-axis (left/right strafe).
 * Target lateral offset: 0.0m (centered on tag).
 */
public class AlignToTagYCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_targetId;

  // Target position
  private static final double TARGET_OFFSET_Y = 0.0; // 0.0m

  // Y-axis controller only
  private final PIDController m_yController;

  public AlignToTagYCommand(DriveSubsystem drive, VisionSubsystem vision, int targetId) {
    m_drive = drive;
    m_vision = vision;
    m_targetId = targetId;

    // Initialize Y controller
    m_yController = new PIDController(
        VisionAlignmentConstants.kVisionYP,
        VisionAlignmentConstants.kVisionYI,
        VisionAlignmentConstants.kVisionYD);
    m_yController.setTolerance(VisionAlignmentConstants.kYTolerance);
    m_yController.disableContinuousInput();

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    System.out.println("AlignToTagYCommand: Testing Y-axis (strafe) control for tag ID " + m_targetId);
    m_yController.reset();
  }

  @Override
  public void execute() {
    var frontLeftTarget = m_vision.getFrontLeftTarget();
    var frontRightTarget = m_vision.getFrontRightTarget();

    if (!frontLeftTarget.isPresent() && !frontRightTarget.isPresent()) {
      m_drive.drive(0, 0, 0, false);
      SmartDashboard.putString("Vision/Status", "No target detected");
      return;
    }

    // Search for the specific target ID from either camera
    var targetFromFrontLeft = m_vision.getFrontLeftTargetById(m_targetId);
    var targetFromFrontRight = m_vision.getFrontRightTargetById(m_targetId);

    if (!targetFromFrontLeft.isPresent() && !targetFromFrontRight.isPresent()) {
      m_drive.drive(0, 0, 0, false);
      SmartDashboard.putString("Vision/Status", "Target " + m_targetId + " not found");
      return;
    }

    boolean useFrontLeft = targetFromFrontLeft.isPresent();
    var target = useFrontLeft ? targetFromFrontLeft : targetFromFrontRight;
    if (target.isEmpty()) {
      m_drive.drive(0, 0, 0, false);
      return;
    }

    var t = target.get();

    // Get measurements - use camera-to-target transformation for accurate distance
    double yaw = t.getYaw();  // Horizontal angle error
    var transform = t.getBestCameraToTarget();
    double cameraDistanceX = transform.getX();  // Forward distance from camera in meters
    
    // Account for camera Y offset from robot center
    // FrontLeft camera is at Y = -trackwidth/2
    // FrontRight camera is at Y = +trackwidth/2
    double cameraOffsetY = useFrontLeft ? (-DriveConstants.kTrackWidth / 2) : (DriveConstants.kTrackWidth / 2);
    
    // Estimate lateral offset from yaw
    double cameraLateralOffset = cameraDistanceX * Math.tan(Math.toRadians(yaw));
    
    // Adjust for camera Y position offset
    // We ADD the offset because we need to correct for where the camera is mounted
    double lateralOffset = cameraLateralOffset + cameraOffsetY;

    // Y-axis control: strafe left/right to center on tag
    double ySpeed = m_yController.calculate(lateralOffset, TARGET_OFFSET_Y);
    ySpeed = MathUtil.clamp(ySpeed, -0.5, 0.5);

    // Stop movement when at setpoint to eliminate jitter
    if (m_yController.atSetpoint()) {
      ySpeed = 0;
    }

    // Only move in Y, no X or rotation
    m_drive.drive(0, ySpeed, 0, false);

    // Telemetry
    SmartDashboard.putNumber("Vision/Robot_Y", lateralOffset);
    SmartDashboard.putString("Vision/Status", "Y-Axis - Tag " + t.getFiducialId());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    SmartDashboard.putString("Vision/Status", interrupted ? "Y-Axis Interrupted" : "Y-Axis Complete");
  }

  @Override
  public boolean isFinished() {
    // Never finish - continuously hold position while button is held
    return false;
  }
}
