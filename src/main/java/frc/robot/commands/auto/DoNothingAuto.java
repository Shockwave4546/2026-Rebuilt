// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Do Nothing autonomous routine. Simply ends immediately without performing any
 * actions.
 */
public class DoNothingAuto extends SequentialCommandGroup {
  /**
   * Creates a new DoNothingAuto command group.
   */
  public DoNothingAuto() {
    addCommands(
        new InstantCommand(() -> System.out.println("Do Nothing Auto started")));
  }
}
