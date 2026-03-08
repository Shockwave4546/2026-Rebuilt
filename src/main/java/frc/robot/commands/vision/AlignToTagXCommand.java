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
 * Vision alignment command that controls only the X-axis (forward/backward distance).
 * Target distance: 1.5m from the tag.
 */
public class AlignToTagXCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final int m_targetId;

  // Target position
  private static final double TARGET_DISTANCE_X = 1.5; // 1.5m

  // X-axis controller only
  private final PIDController m_xController;

  public AlignToTagXCommand(DriveSubsystem drive, VisionSubsystem vision, int targetId) {
    m_drive = drive;
    m_vision = vision;
    m_targetId = targetId;

    // Initialize X controller
    m_xController = new PIDController(
        VisionAlignmentConstants.kVisionXP,
        VisionAlignmentConstants.kVisionXI,
        VisionAlignmentConstants.kVisionXD);
    m_xController.setTolerance(VisionAlignmentConstants.kXTolerance);
    m_xController.disableContinuousInput();

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    System.out.println("AlignToTagXCommand: Testing X-axis (distance) control for tag ID " + m_targetId);
    m_xController.reset();
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
    var transform = t.getBestCameraToTarget();
    double cameraDistanceX = transform.getX();  // Forward distance from camera in meters
    
    // Account for camera offset from robot center
    // Both cameras are at (wheelbase/2, ±trackwidth/2)
    double cameraOffsetX = DriveConstants.kWheelBase / 2;
    
    // Transform from camera frame to robot center frame
    double distanceToTag = cameraDistanceX - cameraOffsetX;

    // X-axis control: forward/backward to reach 1.5m
    double xSpeed = -m_xController.calculate(distanceToTag, TARGET_DISTANCE_X);
    xSpeed = MathUtil.clamp(xSpeed, -0.5, 0.5);

    // Stop movement when at setpoint to eliminate jitter
    if (m_xController.atSetpoint()) {
      xSpeed = 0;
    }

    // Only move in X, no Y or rotation
    m_drive.drive(xSpeed, 0, 0, false);

    // Telemetry
    SmartDashboard.putNumber("Vision/Robot_X", distanceToTag);
    SmartDashboard.putString("Vision/Status", "X-Axis - Tag " + t.getFiducialId());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    SmartDashboard.putString("Vision/Status", interrupted ? "X-Axis Interrupted" : "X-Axis Complete");
  }

  @Override
  public boolean isFinished() {
    // Never finish - continuously hold position while button is held
    return false;
  }
}
