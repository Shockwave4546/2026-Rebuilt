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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.vision.AlignToTagCommand;
import frc.robot.commands.vision.AlignToTagXCommand;
import frc.robot.commands.vision.AlignToTagYCommand;
import frc.robot.commands.vision.AlignToTagHeadingCommand;
import frc.robot.commands.vision.AlignToTagSkewCommand;
import frc.robot.commands.vision.AlignToGoalCommand;
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
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();

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

    // Vision Alignment - Individual Axis Testing
    // X button: Test X-axis (forward/backward distance control)
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new AlignToTagXCommand(m_robotDrive, m_vision, 22));

    // Y button: Test Y-axis (left/right strafe control)
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new AlignToTagYCommand(m_robotDrive, m_vision, 22));

    // B button: Test Heading/Yaw (rotation control based on robot-to-tag angle)
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new AlignToTagHeadingCommand(m_robotDrive, m_vision, 22));

    // LB button: Align to tag skew (square with tag heading)
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new AlignToTagSkewCommand(m_robotDrive, m_vision, 22));

    // RB button: Align to goal (offset from tag)
    // Goal is 0.6m behind and 0.25m right of tag
    // Robot should position at 1.5m away from goal, centered, facing toward goal
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new AlignToGoalCommand(m_robotDrive, m_vision, 22, 
                                          -0.6, .5,    // tag to goal offset (0.6m back, 0.25m right)
                                          1.5, 0.0,      // desired robot offset (1.5m away, centered)
                                          0.0));         // desired yaw (0° = face toward goal)

    // A button: Full XYYaw alignment (all axes)
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new AlignToTagCommand(m_robotDrive, m_vision, 22));

    // Shooter Controls
    // Back button: Shoot short distance
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(
            () -> m_launcher.shootShort(),
            m_launcher))
        .onFalse(new InstantCommand(
            () -> m_launcher.stopLauncher(),
            m_launcher));

    // Start button (if not using for zeroHeading anymore, or using buttons 9-10 for triggers)
    // Using button 9 (Right Trigger): Shoot long distance
    new JoystickButton(m_driverController, 9)  // Right Trigger button ID
        .onTrue(new InstantCommand(
            () -> m_launcher.shootLong(),
            m_launcher))
        .onFalse(new InstantCommand(
            () -> m_launcher.stopLauncher(),
            m_launcher));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoSelector.getSelectedAuto();
  }

  /**
   * Gets the vision subsystem.
   *
   * @return the vision subsystem
   */
  public VisionSubsystem getVisionSubsystem() {
    return m_vision;
  }
}
