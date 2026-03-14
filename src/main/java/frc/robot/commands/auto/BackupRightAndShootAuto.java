// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Autonomous routine that backs up 0.25 meters and shoots with jam-handling logic.
 * 
 * Sequence:
 * 1. Back up 0.25 meters
 * 2. Shoot for 2.5 seconds
 * 3. Wait 1 second
 * 4. Shoot for 2.5 seconds
 * 5. Reverse indexer/feeder for 0.5 seconds to unjam
 * 6. Shoot for 2.5 more seconds
 * 7. Stop
 */
public class BackupRightAndShootAuto extends SequentialCommandGroup {
  /**
   * Creates a new BackUpAndShootAuto command group.
   *
   * @param drive the drive subsystem
   * @param launcher the launcher subsystem
   * @param indexer the indexer subsystem
   */
  public BackupRightAndShootAuto(DriveSubsystem drive, LauncherSubsystem launcher, IndexerSubsystem indexer) {
    addCommands(
        new InstantCommand(() -> System.out.println("Back Up and Shoot Auto started")),

        // Back up 0.25 meters at 5% speed
        new DriveToPositionCommand(drive, -1.33, 0.1),

        // First shoot cycle: 2.5 seconds
        new InstantCommand(() -> {
          System.out.println("Starting first shoot cycle (3.5s)");
          launcher.shootLong();
        }),
        new WaitCommand(3.5),
        new InstantCommand(() -> launcher.stopLauncher()),

        // Wait 1 second
        new InstantCommand(() -> System.out.println("Waiting 1 second between shots")),
        new WaitCommand(1.0),

        // Second shoot cycle: 2.5 seconds
        new InstantCommand(() -> {
          System.out.println("Starting second shoot cycle (3.5s)");
          launcher.shootLong();
        }),
        new WaitCommand(3.5),
        new InstantCommand(() -> launcher.stopLauncher()),

        // Wait 1 second
        new InstantCommand(() -> System.out.println("Waiting 1 second between shots")),
        new WaitCommand(1.0),

        // Unjam: reverse indexer and feeder for 0.75 seconds
        new InstantCommand(() -> {
          System.out.println("Starting unjam sequence (0.75s)");
          indexer.runReverse();
          launcher.runFeederReverse();
        }),
        new WaitCommand(0.75),
        new InstantCommand(() -> {
          indexer.stop();
          launcher.stopFeeder();
        }),

        // Third shoot cycle: 2.5 seconds
        new InstantCommand(() -> {
          System.out.println("Starting third shoot cycle (4s)");
          launcher.shootLong();
        }),
        new WaitCommand(4),
        new InstantCommand(() -> launcher.stopLauncher()),

               // Wait 1 second
        new InstantCommand(() -> System.out.println("Waiting 1 second between shots")),
        new WaitCommand(1.0),

        // Unjam: reverse indexer and feeder for 0.75 seconds
        new InstantCommand(() -> {
          System.out.println("Starting unjam sequence (0.75s)");
          indexer.runReverse();
          launcher.runFeederReverse();
        }),
        new WaitCommand(0.75),
        new InstantCommand(() -> {
          indexer.stop();
          launcher.stopFeeder();
        }),

        // Third shoot cycle: 2.5 seconds
        new InstantCommand(() -> {
          System.out.println("Starting third shoot cycle (4s)");
          launcher.shootLong();
        }),
        new WaitCommand(4),
        new InstantCommand(() -> launcher.stopLauncher()),

        // Final stop
        new InstantCommand(() -> {
          System.out.println("Back Up and Shoot Auto complete");
          drive.drive(0, 0, 0, false);
        }));
  }
}
