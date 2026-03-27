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
 * Snap180HeadingCommand snaps the robot to the closest heading: 0° or 180°.
 * Useful for quick alignment without complex multi-angle snapping.
 * 
 * - **Snap behavior**: Automatically rotates to nearest 0° or 180° when command starts
 * - **Hold behavior**: Maintains heading while holding the button, allows translation
 */
public class Snap180HeadingCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  
  private final PIDController m_headingController;
  private double m_targetHeading = 0;

  /**
   * Creates a new Snap180HeadingCommand.
   *
   * @param driveSubsystem The drive subsystem
   * @param xSpeedSupplier Supplier for forward/backward speed
   * @param ySpeedSupplier Supplier for left/right speed
   */
  public Snap180HeadingCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier) {
    m_driveSubsystem = driveSubsystem;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;

    // Create PID controller for heading
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
    // Snap to the closest of 0° or 180°
    double currentHeading = m_driveSubsystem.getHeading();
    
    // Normalize to -180 to 180 (already in this range from Pigeon)
    double normalized = MathUtil.angleModulus(currentHeading);
    
    // Calculate shortest distance to each target
    double absNorm = Math.abs(normalized);
    
    // Distance to 0°
    double distanceTo0 = absNorm;
    
    // Distance to 180°
    double distanceTo180 = 180 - absNorm;
    
    // Snap to whichever is closer
    if (distanceTo0 <= distanceTo180) {
      m_targetHeading = 0;
    } else {
      m_targetHeading = 180;
    }
    
    SmartDashboard.putNumber("Snap180/Initial Snap", m_targetHeading);
  }

  @Override
  public void execute() {
    double currentHeading = m_driveSubsystem.getHeading();
    
    // Normalize target to match the current heading's range to avoid wraparound issues
    // If target is 180 but current is -170, we want to use -180 as the target instead
    double normalizedTarget = m_targetHeading;
    if (m_targetHeading == 180 && currentHeading < 0) {
      normalizedTarget = -180;
    }
    
    // Calculate error for feed-forward
    double error = normalizedTarget - currentHeading;
    
    // Calculate PID output for heading control
    double pidOutput = m_headingController.calculate(currentHeading, normalizedTarget);
    
    // Add feed-forward to boost response speed
    double ffOutput = Math.copySign(HeadingControllerConstants.kHeadingFF, error);
    
    // Combine PID and feed-forward
    double rotationOutput = pidOutput + ffOutput;
    
    // Clamp rotation output
    rotationOutput = MathUtil.clamp(rotationOutput, -1.0, 1.0);
    
    // Drive with translation from suppliers and calculated rotation
    m_driveSubsystem.drive(
        m_xSpeedSupplier.getAsDouble(),
        m_ySpeedSupplier.getAsDouble(),
        rotationOutput,
        true   // Field-relative
    );

    SmartDashboard.putNumber("Snap180/Target", m_targetHeading);
    SmartDashboard.putNumber("Snap180/Normalized Target", normalizedTarget);
    SmartDashboard.putNumber("Snap180/Current", currentHeading);
    SmartDashboard.putNumber("Snap180/Error", error);
    SmartDashboard.putNumber("Snap180/Rotation Output", rotationOutput);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;  // Command runs while button is held (whileTrue)
  }
}
