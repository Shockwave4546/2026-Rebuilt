// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Rotates the robot to a target rotation angle.
 * Uses the gyro to track rotation.
 */
public class RotateToAngleCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_targetAngle; // Target angle in degrees
  private final double m_rotationSpeed; // Rotation speed as a fraction from -1.0 to 1.0
  private double m_startingAngle;
  private static final double ANGLE_TOLERANCE = 2.0; // degrees

  /**
   * Creates a new RotateToAngleCommand.
   *
   * @param drive the drive subsystem
   * @param targetAngle the target rotation angle in degrees (positive = counterclockwise)
   * @param rotationSpeed the rotation speed from -1.0 to 1.0
   */
  public RotateToAngleCommand(DriveSubsystem drive, double targetAngle, double rotationSpeed) {
    m_drive = drive;
    m_targetAngle = targetAngle;
    m_rotationSpeed = rotationSpeed;
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_startingAngle = m_drive.getPose().getRotation().getDegrees();
    System.out.println("RotateToAngleCommand started. Current angle: " + m_startingAngle + ", Target: " + m_targetAngle);
  }

  @Override
  public void execute() {
    m_drive.drive(0, 0, m_rotationSpeed*.05, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    
    double endingAngle = m_drive.getPose().getRotation().getDegrees();
    System.out.println("RotateToAngleCommand ended. Target: " + m_targetAngle + ", Actual: " + endingAngle);
  }

  @Override
  public boolean isFinished() {
    double currentAngle = m_drive.getPose().getRotation().getDegrees();
    
    // Calculate the shortest angle difference
    double angleDiff = currentAngle - m_targetAngle;
    // Normalize to -180 to 180
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    return Math.abs(angleDiff) <= ANGLE_TOLERANCE;
  }
}
