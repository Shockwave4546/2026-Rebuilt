// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoordinatedHeadingCommand;
import frc.robot.commands.HoldHeadingCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystemProfiled;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final IntakeSubsystemProfiled m_intake = new IntakeSubsystemProfiled();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();

  // The autonomous selector
  private final AutoSelector m_autoSelector = new AutoSelector(m_robotDrive);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Track intake state for toggle functionality
  private boolean m_intakePivotToggleState = false; // false = retracted, true = deployed
  private boolean m_intakeRollerToggleState = false; // false = off, true = on

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize dashboard controls
    SmartDashboard.putNumber("Speed Multiplier", 1.0);

    // Wire up indexer to launcher for automatic feeding
    m_launcher.setIndexer(m_indexer);

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
    // Start button: Reset heading to 0 degrees
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // X button: Toggle intake up/down (deploy/retract pivot)
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(
            () -> {
              m_intakePivotToggleState = !m_intakePivotToggleState;
              if (m_intakePivotToggleState) {
                // Deploy intake
                m_intake.setTargetPosition(IntakeConstants.kIntakePivotDeployedPosition);
              } else {
                // Retract intake
                m_intake.setTargetPosition(IntakeConstants.kIntakePivotRetractedPosition);
              }
            },
            m_intake));

    // Y button: Reverse indexer + feeder (hold to unjam)
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(
            () -> {
              m_indexer.runReverse();
              m_launcher.runFeederReverse();
            },
            m_indexer, m_launcher))
        .onFalse(new InstantCommand(
            () -> {
              m_indexer.stop();
              m_launcher.stopFeeder();
            },
            m_indexer, m_launcher));
    // A button: Coordinated heading (hold) — intake faces direction of travel
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new CoordinatedHeadingCommand(
            m_robotDrive,
            () -> DriveSubsystem.applyThrustExpo(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)),
            () -> DriveSubsystem.applyThrustExpo(-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)),
            () -> DriveSubsystem.applyThrustExpo(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband))));

    // B button: Hold 45 degrees yaw
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
         .whileTrue(new HoldHeadingCommand(m_robotDrive,
             () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
             () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
             45.0));

    // RB button: Toggle intake rollers on/off
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(
            () -> {
              m_intakeRollerToggleState = !m_intakeRollerToggleState;
              if (m_intakeRollerToggleState) {
                m_intake.run();  // Turn rollers on
              } else {
                m_intake.stopRollers();  // Turn rollers off
              }
            },
            m_intake));

    // RT (Right Trigger) - Analog axis 3: Intake reverse (hold)
    // Triggers are analog (0-1), not digital buttons
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.1)
        .onTrue(new InstantCommand(
            () -> m_intake.reverseRollers(),
            m_intake))
        .onFalse(new InstantCommand(
            () -> m_intake.stopRollers(),
            m_intake));

    // LB button: Interrupt intake position controller (safety stop for intake arm)
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(
            () -> m_intake.stop(),  // Stops all intake control (pivot + rollers)
            m_intake));

    // LT (Left Trigger): Shoot (hold) - spins up shooter, feeds + indexes when at target RPM
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.1)
        .onTrue(new InstantCommand(
            () -> m_launcher.shootShort(),
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
