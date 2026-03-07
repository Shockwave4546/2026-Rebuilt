// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Autonomous routine that performs a 90-degree rotation in place.
 * Uses gyro-based rotation tracking for accurate positioning.
 */
public class RotateAuto extends SequentialCommandGroup {
  /**
   * Creates a new RotateAuto command group.
   *
   * @param drive the drive subsystem
   */
  public RotateAuto(DriveSubsystem drive) {
    addCommands(
        new InstantCommand(() -> System.out.println("Rotate Auto started")),
        // Rotate 90 degrees counterclockwise at 50% rotation speed
        new RotateToAngleCommand(drive, 90.0, 0.5),
        // Stop
        new InstantCommand(() -> drive.drive(0, 0, 0, false)));
  }
}
