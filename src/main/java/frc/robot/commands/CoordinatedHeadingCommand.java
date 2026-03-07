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

/**
 * CoordinatedHeadingCommand keeps the robot's front (intake) facing the direction of translation.
 * 
 * The driver can move freely with the translation stick, and the rotation controller automatically
 * orients the robot so the intake faces the direction of travel. When translation stops, the robot
 * holds the last known translation direction. This is useful for maintaining proper intake 
 * orientation during collection operations.
 */
public class CoordinatedHeadingCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rotationOverrideSupplier;
  private final PIDController m_headingController;
  
  // Deadband to prevent jitter when joystick is near center
  private final double m_translationDeadband = 0.01;
  
  // Remember the last translation direction for holding when stopped
  private double m_lastTargetHeading = 0;

  /**
   * Calculates a speed multiplier based on heading error using exponential scaling.
   * At 0° error: full speed (1.0)
   * At 10° error: 50% speed (0.5)
   * At 180° error: no speed (0.0)
   * Uses exponential curve: 2^(-error/10) for aggressive speed limiting
   * 
   * @param headingError The current heading error in degrees
   * @return Speed multiplier from 0.0 to 1.0
   */
  private double calculateSpeedScaling(double headingError) {
    // Normalize error to 0-180 range (absolute value of shortest path)
    double absError = Math.abs(headingError);
    if (absError > 180) {
      absError = 360 - absError;
    }
    
    // Use exponential decay: 2^(-error/10)
    // At 0°: 2^0 = 1.0 (full speed)
    // At 10°: 2^-1 = 0.5 (half speed)
    // At 180°: 2^-18 ≈ 0.0000038 (essentially zero)
    double scalingFactor = Math.pow(2.0, -absError / 10.0);
    
    // Clamp to ensure we never go negative or exceed 1.0
    return MathUtil.clamp(scalingFactor, 0.0, 1.0);
  }

  /**
   * Creates a new CoordinatedHeadingCommand.
   *
   * @param driveSubsystem The drive subsystem
   * @param xSpeedSupplier Supplier for forward/backward speed (field-relative)
   * @param ySpeedSupplier Supplier for left/right speed (field-relative)
   * @param rotationOverrideSupplier Supplier for manual rotation override (if needed)
   */
  public CoordinatedHeadingCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotationOverrideSupplier) {
    m_driveSubsystem = driveSubsystem;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;
    m_rotationOverrideSupplier = rotationOverrideSupplier;

    // Create PID controller for heading with configurable gains
    m_headingController = new PIDController(
        HeadingControllerConstants.kHeadingP,
        HeadingControllerConstants.kHeadingI,
        HeadingControllerConstants.kHeadingD);
    m_headingController.enableContinuousInput(-180, 180);
    m_headingController.setTolerance(HeadingControllerConstants.kHeadingTolerance);

    addRequirements(driveSubsystem);
  }

  /**
   * Calculates the target heading based on translation direction.
   * If there's significant translation, updates the target heading.
   * If no translation, returns the last known translation direction.
   * 
   * @param xSpeed Forward/backward speed
   * @param ySpeed Left/right speed
   * @return Target heading in degrees (-180 to 180)
   */
  private double calculateTargetHeadingFromTranslation(double xSpeed, double ySpeed) {
    // Calculate the magnitude of translation first
    double translationMagnitude = Math.hypot(xSpeed, ySpeed);
    
    // If no significant translation input, hold the last known direction
    if (translationMagnitude < m_translationDeadband) {
      return m_lastTargetHeading;
    }

    // Calculate angle from translation inputs
    // atan2(y, x) gives angle in radians from -π to π
    // Note: In FRC coordinate system, +x is forward, +y is left
    // We want the heading that points in the direction of travel
    double angleRadians = Math.atan2(ySpeed, xSpeed);
    double angleDegrees = Math.toDegrees(angleRadians);

    // Normalize to -180 to 180 range
    while (angleDegrees > 180) {
      angleDegrees -= 360;
    }
    while (angleDegrees < -180) {
      angleDegrees += 360;
    }

    // Update last known direction for when translation stops
    m_lastTargetHeading = angleDegrees;
    
    return angleDegrees;
  }

  @Override
  public void initialize() {
    // Reset the PID controller when command starts
    m_headingController.reset();
    // Set initial target heading to current heading
    m_lastTargetHeading = m_driveSubsystem.getHeading();
  }

  @Override
  public void execute() {
    // Get speed multiplier from dashboard
    double speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", 1.0);

    // Get joystick inputs
    double xSpeed = m_xSpeedSupplier.getAsDouble();
    double ySpeed = m_ySpeedSupplier.getAsDouble();
    double rotationOverride = m_rotationOverrideSupplier.getAsDouble();

    // Calculate target heading based on translation direction
    double targetHeading = calculateTargetHeadingFromTranslation(xSpeed, ySpeed);
    
    // Calculate heading error
    double headingError = targetHeading - m_driveSubsystem.getHeading();
    
    // Normalize error to -180 to 180 range
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;

    // Calculate rotation correction using PID
    double pidOutput = m_headingController.calculate(
        m_driveSubsystem.getHeading(),
        targetHeading);
    
    // Add feed-forward to PID output for faster response
    double feedForwardOutput = headingError * HeadingControllerConstants.kHeadingFF;
    
    // Combine PID and feed-forward outputs
    double rawRotationOutput = pidOutput + feedForwardOutput;

    // If driver is providing significant rotation input, blend with override
    // This allows the driver to manually adjust rotation if needed
    double rotationCorrection;
    if (Math.abs(rotationOverride) > 0.05) {
      // Driver is providing rotation input, use that instead
      rotationCorrection = rotationOverride;
      // Reset PID to avoid windup when manual control takes over
      m_headingController.reset();
    } else {
      // Clamp the rotation correction to max speed
      rotationCorrection = MathUtil.clamp(rawRotationOutput, 
          -HeadingControllerConstants.kMaxHeadingRotationSpeed, 
          HeadingControllerConstants.kMaxHeadingRotationSpeed);
    }
    
    // Calculate speed scaling based on heading error
    // This allows smooth deceleration as we turn to face the desired direction
    double speedScaling = calculateSpeedScaling(headingError);

    // Drive with coordinated heading and speed scaling
    m_driveSubsystem.drive(
        xSpeed * speedMultiplier * speedScaling,
        ySpeed * speedMultiplier * speedScaling,
        rotationCorrection,
        true);

    // Log to dashboard for debugging
    SmartDashboard.putNumber("Target Heading (Coordinated)", targetHeading);
    SmartDashboard.putNumber("Current Heading", m_driveSubsystem.getHeading());
    SmartDashboard.putNumber("Heading Error (Coordinated)", headingError);
    SmartDashboard.putNumber("PID Output (Coordinated)", pidOutput);
    SmartDashboard.putNumber("FF Output (Coordinated)", feedForwardOutput);
    SmartDashboard.putNumber("Speed Scaling Factor", speedScaling);
    SmartDashboard.putNumber("Translation Magnitude", Math.hypot(xSpeed, ySpeed));
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
