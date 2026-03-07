// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Drives the robot forward to a target distance from the starting position.
 * Uses odometry to track distance traveled.
 */
public class DriveToPositionCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_targetDistance; // Distance in meters
  private final double m_speed; // Speed as a fraction from -1.0 to 1.0
  private Pose2d m_startingPose;

  /**
   * Creates a new DriveToPositionCommand.
   *
   * @param drive the drive subsystem
   * @param targetDistance the target distance to travel in meters (positive = forward, negative = backward)
   * @param speed the speed to drive at from -1.0 to 1.0
   */
  public DriveToPositionCommand(DriveSubsystem drive, double targetDistance, double speed) {
    m_drive = drive;
    m_targetDistance = targetDistance;
    m_speed = Math.abs(speed); // Ensure positive, direction handled by targetDistance sign
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Record the starting position
    m_startingPose = m_drive.getPose();
    System.out.println("DriveToPositionCommand started at pose: " + m_startingPose);
  }

  @Override
  public void execute() {
    // Drive in the specified direction
    if (m_targetDistance >= 0) {
      m_drive.drive(m_speed, 0, 0, false);
    } else {
      m_drive.drive(-m_speed, 0, 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_drive.drive(0, 0, 0, false);
    
    Pose2d endingPose = m_drive.getPose();
    double distanceTraveled = endingPose.getTranslation().getDistance(m_startingPose.getTranslation());
    System.out.println("DriveToPositionCommand ended. Target: " + m_targetDistance + "m, Actual: " + distanceTraveled + "m");
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = m_drive.getPose();
    double distanceTraveled = currentPose.getTranslation().getDistance(m_startingPose.getTranslation());
    
    // Check if we've reached the target distance
    return Math.abs(distanceTraveled) >= Math.abs(m_targetDistance);
  }
}
