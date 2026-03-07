// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class HoldHeadingCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final double m_targetHeading;
  private final PIDController m_headingController;
  private final double m_maxRotationSpeed = 0.1; // Maximum rotation speed (0.0 to 1.0)

  /**
   * Creates a new HoldHeadingCommand.
   *
   * @param driveSubsystem The drive subsystem
   * @param xSpeedSupplier Supplier for forward/backward speed
   * @param ySpeedSupplier Supplier for left/right speed
   * @param targetHeading The target heading to hold (in degrees)
   */
  public HoldHeadingCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      double targetHeading) {
    m_driveSubsystem = driveSubsystem;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;
    m_targetHeading = targetHeading;

    // Create PID controller for heading
    // Starting with very small P and no D to test behavior
    m_headingController = new PIDController(0.001, 0.0, 0.00);
    m_headingController.enableContinuousInput(-180, 180);
    m_headingController.setTolerance(2.0);

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    // Reset the PID controller when command starts
    m_headingController.reset();
  }

  @Override
  public void execute() {
    // Get speed multiplier from dashboard
    double speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", 1.0);

    // Get joystick inputs
    double xSpeed = m_xSpeedSupplier.getAsDouble();
    double ySpeed = m_ySpeedSupplier.getAsDouble();

    // Calculate rotation correction using PID
    double rawPIDOutput = m_headingController.calculate(
        m_driveSubsystem.getHeading(), 
        m_targetHeading);
    
    // Clamp the rotation correction to max speed
    double rotationCorrection = MathUtil.clamp(rawPIDOutput, -m_maxRotationSpeed, m_maxRotationSpeed);

    // Drive with heading hold
    m_driveSubsystem.drive(
        xSpeed * speedMultiplier,
        ySpeed * speedMultiplier,
        rotationCorrection,
        true);

    // Log to dashboard for debugging
    SmartDashboard.putNumber("Target Heading", m_targetHeading);
    SmartDashboard.putNumber("Heading Error", m_targetHeading - m_driveSubsystem.getHeading());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
