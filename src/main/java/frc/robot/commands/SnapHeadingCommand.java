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
 * SnapHeadingCommand snaps the robot to the closest cardinal-diagonal heading (45°, 135°, 225°, 315°),
 * then allows fine-tuning rotation via right stick quadrants.
 * 
 * - **Snap behavior**: Automatically rotates to the nearest 45° angle when command starts
 * - **Right stick control**: 
 *   - Quadrant 1 (↗): 45°
 *   - Quadrant 2 (↖): 135°
 *   - Quadrant 3 (↙): 225°
 *   - Quadrant 4 (↘): 315°
 *   - Center (deadband): Hold last selected angle
 * 
 * Translation stick remains independent and free, with speed control still managed
 * by the drive system's speed multiplier.
 */
public class SnapHeadingCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rightStickXSupplier;
  private final DoubleSupplier m_rightStickYSupplier;
  
  private final PIDController m_headingController;
  
  // Snap angles: 45°, 135°, 225°, 315°
  private static final double[] SNAP_ANGLES = {45, 135, 225, 315};
  
  // Right stick deadband to prevent jitter
  private static final double STICK_DEADBAND = 0.1;
  
  // Current target heading
  private double m_targetHeading = 0;

  /**
   * Creates a new SnapHeadingCommand.
   *
   * @param driveSubsystem The drive subsystem
   * @param xSpeedSupplier Supplier for forward/backward speed
   * @param ySpeedSupplier Supplier for left/right speed
   * @param rightStickXSupplier Supplier for right stick X (rotation)
   * @param rightStickYSupplier Supplier for right stick Y (rotation)
   */
  public SnapHeadingCommand(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rightStickXSupplier,
      DoubleSupplier rightStickYSupplier) {
    m_driveSubsystem = driveSubsystem;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;
    m_rightStickXSupplier = rightStickXSupplier;
    m_rightStickYSupplier = rightStickYSupplier;

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
    // Snap to the closest 45° angle on command start
    m_targetHeading = snapToClosestAngle(m_driveSubsystem.getHeading());
    
    SmartDashboard.putNumber("SnapHeading/Initial Snap", m_targetHeading);
  }

  @Override
  public void execute() {
    // Get current heading
    double currentHeading = m_driveSubsystem.getHeading();
    
    // Get right stick input with deadband applied
    double rightStickX = MathUtil.applyDeadband(m_rightStickXSupplier.getAsDouble(), STICK_DEADBAND);
    double rightStickY = MathUtil.applyDeadband(m_rightStickYSupplier.getAsDouble(), STICK_DEADBAND);
    
    // If right stick is moved beyond deadband, select angle based on quadrant
    if (rightStickX != 0 || rightStickY != 0) {
      // Determine which quadrant the stick is in
      // Note: Y-axis is typically inverted in joysticks (negative = up)
      if (rightStickX >= 0 && -rightStickY >= 0) {
        // Quadrant 1 (↗): 45°
        m_targetHeading = 45;
      } else if (rightStickX < 0 && -rightStickY >= 0) {
        // Quadrant 2 (↖): 135°
        m_targetHeading = 135;
      } else if (rightStickX < 0 && -rightStickY < 0) {
        // Quadrant 3 (↙): 225°
        m_targetHeading = 225;
      } else {
        // Quadrant 4 (↘): 315°
        m_targetHeading = 315;
      }
    }
    // If stick is in deadband, target heading stays the same (hold last selected angle)
    
    // Calculate rotation using PID controller
    double rotationOutput = m_headingController.calculate(currentHeading, m_targetHeading);
    
    // Get translation from driver
    double xSpeed = MathUtil.applyDeadband(m_xSpeedSupplier.getAsDouble(), 0.02);
    double ySpeed = MathUtil.applyDeadband(m_ySpeedSupplier.getAsDouble(), 0.02);
    
    // Drive with calculated rotation and translation
    m_driveSubsystem.drive(xSpeed, ySpeed, rotationOutput, true);
    
    // Dashboard telemetry
    SmartDashboard.putNumber("SnapHeading/Target", m_targetHeading);
    SmartDashboard.putNumber("SnapHeading/Current", currentHeading);
    SmartDashboard.putNumber("SnapHeading/Error", m_headingController.getPositionError());
    SmartDashboard.putNumber("SnapHeading/Rotation Output", rotationOutput);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true);
  }

  /**
   * Finds the closest snap angle to the given heading.
   * 
   * @param currentHeading The current heading in degrees
   * @return The closest snap angle (45, 135, 225, or 315)
   */
  private double snapToClosestAngle(double currentHeading) {
    // Normalize to 0-360 range for easier comparison
    double normalized = ((currentHeading % 360) + 360) % 360;
    
    double closestAngle = SNAP_ANGLES[0];
    double closestDistance = Double.MAX_VALUE;
    
    for (double angle : SNAP_ANGLES) {
      // Calculate shortest angular distance
      double diff = Math.abs(normalized - angle);
      if (diff > 180) {
        diff = 360 - diff;
      }
      
      if (diff < closestDistance) {
        closestDistance = diff;
        closestAngle = angle;
      }
    }
    
    return closestAngle;
  }

  @Override
  public boolean isFinished() {
    return false; // Command runs until interrupted
  }
}
