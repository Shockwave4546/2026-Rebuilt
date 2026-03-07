// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple autonomous routine that drives forward 2 meters based on odometry.
 * Uses position tracking for reliable autonomous movement regardless of battery voltage.
 */
public class SimpleDriveAuto extends SequentialCommandGroup {
  /**
   * Creates a new SimpleDriveAuto command group.
   *
   * @param drive the drive subsystem
   */
  public SimpleDriveAuto(DriveSubsystem drive) {
    addCommands(
        new InstantCommand(() -> System.out.println("Simple Drive Auto started")),
        // Drive forward 2 meters at 50% speed
        new DriveToPositionCommand(drive, 2.0, 0.05),
        // Stop
        new InstantCommand(() -> drive.drive(0, 0, 0, false)));
  }
}
