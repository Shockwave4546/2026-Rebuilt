// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.BackUpAuto;
import frc.robot.commands.auto.DoNothingAuto;
import frc.robot.commands.auto.RotateAuto;
import frc.robot.commands.auto.SimpleDriveAuto;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Manages autonomous mode selection. Provides a SendableChooser on SmartDashboard
 * to allow drivers to select which autonomous routine to run.
 */
public class AutoSelector {
  private SendableChooser<Command> m_autoChooser;
  private final DriveSubsystem m_drive;

  /**
   * Creates the autonomous command chooser and adds all available autonomous
   * routines to it.
   *
   * @param drive the drive subsystem to pass to auto commands
   */
  public AutoSelector(DriveSubsystem drive) {
    m_drive = drive;
    m_autoChooser = new SendableChooser<>();

    // Add autonomous routines
    m_autoChooser.setDefaultOption("Do Nothing", new DoNothingAuto());
    m_autoChooser.addOption("Simple Drive Forward", new SimpleDriveAuto(m_drive));
    m_autoChooser.addOption("Back Up", new BackUpAuto(m_drive));
    m_autoChooser.addOption("Rotate 90 Degrees", new RotateAuto(m_drive));

    // Put the chooser on the SmartDashboard
    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  /**
   * Gets the currently selected autonomous command.
   *
   * @return the selected autonomous command
   */
  public Command getSelectedAuto() {
    return m_autoChooser.getSelected();
  }
}
