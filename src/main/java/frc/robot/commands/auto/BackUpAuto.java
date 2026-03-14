// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Autonomous routine that backs up 1 meter from the starting position.
 * Uses position tracking for reliable movement.
 */
public class BackUpAuto extends SequentialCommandGroup {
  /**
   * Creates a new BackUpAuto command group.
   *
   * @param drive the drive subsystem
   */
  public BackUpAuto(DriveSubsystem drive) {
    addCommands(
        new InstantCommand(() -> System.out.println("Back Up Auto started")),
        // Drive backward 1 meter at 50% speed
        new DriveToPositionCommand(drive, -0.5, 0.05),
        // Stop
        new InstantCommand(() -> drive.drive(0, 0, 0, false)));
  }
}
