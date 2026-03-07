// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.HoldHeadingCommand;
import frc.robot.commands.CoordinatedHeadingCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The autonomous selector
  private final AutoSelector m_autoSelector = new AutoSelector(m_robotDrive);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize dashboard controls
    SmartDashboard.putNumber("Speed Multiplier", 1.0);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // Thrust expo is applied for fine control at low inputs while maintaining full speed.
        new RunCommand(
            () -> m_robotDrive.drive(
                DriveSubsystem.applyThrustExpo(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)),
                DriveSubsystem.applyThrustExpo(-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)),
                DriveSubsystem.applyThrustExpo(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new HoldHeadingCommand(
            m_robotDrive,
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            0));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new CoordinatedHeadingCommand(
            m_robotDrive,
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoSelector.getSelectedAuto();
  }
}
