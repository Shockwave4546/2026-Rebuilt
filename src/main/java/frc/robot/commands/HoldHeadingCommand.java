// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.HeadingControllerConstants;

import java.util.function.DoubleSupplier;

public class HoldHeadingCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final double m_targetHeading;
  private final PIDController m_headingController;

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

    // Create PID controller for heading with configurable gains
    m_headingController = new PIDController(
        HeadingControllerConstants.kHeadingP,
        HeadingControllerConstants.kHeadingI,
        HeadingControllerConstants.kHeadingD);
    m_headingController.enableContinuousInput(-180, 180);
    m_headingController.setTolerance(HeadingControllerConstants.kHeadingTolerance);

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

    // Calculate heading error
    double headingError = m_targetHeading - m_driveSubsystem.getHeading();
    
    // Normalize error to -180 to 180 range
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;

    // Calculate rotation correction using PID
    double pidOutput = m_headingController.calculate(
        m_driveSubsystem.getHeading(), 
        m_targetHeading);
    
    // Add feed-forward to PID output for faster response
    // FF is proportional to the heading error
    double feedForwardOutput = headingError * HeadingControllerConstants.kHeadingFF;
    
    // Combine PID and feed-forward outputs
    double rawRotationOutput = pidOutput + feedForwardOutput;
    
    // Clamp the rotation correction to max speed
    double rotationCorrection = MathUtil.clamp(rawRotationOutput, 
        -HeadingControllerConstants.kMaxHeadingRotationSpeed, 
        HeadingControllerConstants.kMaxHeadingRotationSpeed);

    // Drive with heading hold
    m_driveSubsystem.drive(
        xSpeed * speedMultiplier,
        ySpeed * speedMultiplier,
        rotationCorrection,
        true);

    // Log to dashboard for debugging
    SmartDashboard.putNumber("Target Heading", m_targetHeading);
    SmartDashboard.putNumber("Heading Error", headingError);
    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("FF Output", feedForwardOutput);
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
