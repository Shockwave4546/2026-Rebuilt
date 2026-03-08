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
 * Vision alignment command that controls only the Yaw-axis (rotation/heading).
 * Target yaw: 0.0 degrees (facing the tag).
 */
public class AlignToTagHeadingCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_targetId;

  // Target position
  private static final double TARGET_YAW = 0.0; // 0.0 degrees

  // Yaw-axis controller only
  private final PIDController m_yawController;

  public AlignToTagHeadingCommand(DriveSubsystem drive, VisionSubsystem vision, int targetId) {
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
    System.out.println("AlignToTagHeadingCommand: Testing Heading control for tag ID " + m_targetId);
    m_yawController.reset();
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

    var target = targetFromFrontLeft.isPresent() ? targetFromFrontLeft : targetFromFrontRight;
    if (target.isEmpty()) {
      m_drive.drive(0, 0, 0, false);
      return;
    }

    var t = target.get();

    // Get measurements
    boolean useFrontLeft = targetFromFrontLeft.isPresent();
    double cameraYaw = t.getYaw();
    
    var transform = t.getBestCameraToTarget();
    double cameraDistanceX = transform.getX();
    
    // Camera offset from robot center
    double cameraOffsetY = useFrontLeft ? (-DriveConstants.kTrackWidth / 2) : (DriveConstants.kTrackWidth / 2);
    
    // Calculate angular offset caused by camera position
    double angularOffsetRad = Math.atan2(cameraOffsetY, cameraDistanceX);
    double angularOffsetDeg = Math.toDegrees(angularOffsetRad);
    
    // Correct the yaw to get robot-to-target yaw
    // (We add because the offset correction works the same way as the lateral offset)
    double robotYaw = cameraYaw + angularOffsetDeg;

    // Yaw control: rotate to face the tag using robot-to-target yaw
    double rotSpeed = m_yawController.calculate(robotYaw, TARGET_YAW);
    rotSpeed = MathUtil.clamp(rotSpeed, -0.5, 0.5);

    // Stop movement when at setpoint to eliminate jitter
    if (m_yawController.atSetpoint()) {
      rotSpeed = 0;
    }

    // Only rotate, no X or Y movement
    m_drive.drive(0, 0, rotSpeed, false);

    // Telemetry
    SmartDashboard.putNumber("Vision/Robot_Yaw", robotYaw);
    SmartDashboard.putString("Vision/Status", "Heading - Tag " + t.getFiducialId());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    SmartDashboard.putString("Vision/Status", interrupted ? "Heading Interrupted" : "Heading Complete");
  }

  @Override
  public boolean isFinished() {
    // Never finish - continuously hold position while button is held
    return false;
  }
}
