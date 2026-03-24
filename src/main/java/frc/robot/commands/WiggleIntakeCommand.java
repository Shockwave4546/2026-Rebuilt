// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystemProfiled;

/**
 * Wiggles the intake pivot between fully deployed and halfway toward retracted.
 * 
 * <p>Purpose: Shuffle leftover game pieces from the intake hopper into the indexer.
 * 
 * <p>Motion profile:
 * - Low position:  0.29 rotations (fully deployed, all the way to the ground)
 * - High position: ~0.46 rotations (halfway up toward retracted)
 * - Strategy: Simple setpoint swapping — when arm reaches low, command it to high, and vice versa.
 *   No trapezoidal profile, just direct PID control for snappy motion.
 * 
 * <p>The command completes after the specified number of complete oscillation cycles.
 * Returns to fully deployed position when finished.
 */
public class WiggleIntakeCommand extends Command {
  private final IntakeSubsystemProfiled m_intake;
  
  // Wiggle parameters
  private static final double DEPLOYED_POSITION = IntakeConstants.kIntakePivotDeployedPosition;  // 0.29
  private static final double RETRACTED_POSITION = IntakeConstants.kIntakePivotRetractedPosition; // 0.63
  
  // Calculate halfway from deployed to retracted
  private static final double HALFWAY_UP = DEPLOYED_POSITION + ((RETRACTED_POSITION - DEPLOYED_POSITION) / 2.0);  // ~0.46
  
  private static final double LOW_POSITION = DEPLOYED_POSITION;   // All the way to the ground
  private static final double HIGH_POSITION = HALFWAY_UP;          // Halfway up
  
  // Position tolerance for detecting when we've reached a setpoint (rotations)
  private static final double POSITION_TOLERANCE = 0.02;  // ~7 degrees
  
  // Number of complete oscillations to perform
  private final int m_cycleCount;
  
  // State tracking
  private int m_completedCycles = 0;
  private boolean m_movingUp = false; // Start by moving to low position, then up
  
  /**
   * Creates a new WiggleIntakeCommand.
   * 
   * @param intake The intake subsystem
   * @param cycles Number of complete oscillation cycles to perform
   */
  public WiggleIntakeCommand(IntakeSubsystemProfiled intake, int cycles) {
    m_intake = intake;
    m_cycleCount = cycles;
    addRequirements(m_intake);
  }
  
  /**
   * Convenience constructor: default 3 cycles
   */
  public WiggleIntakeCommand(IntakeSubsystemProfiled intake) {
    this(intake, 3);
  }

  @Override
  public void initialize() {
    m_completedCycles = 0;
    m_movingUp = false;
    
    // Start at low position
    m_intake.setTargetPosition(LOW_POSITION);
    
    SmartDashboard.putString("Wiggle/Status", "Starting");
    SmartDashboard.putNumber("Wiggle/Cycle", 0);
  }

  @Override
  public void execute() {
    double currentPos = m_intake.getEncoderPosition();
    double targetPos = m_movingUp ? HIGH_POSITION : LOW_POSITION;
    
    // Check if we've reached the current setpoint
    if (Math.abs(currentPos - targetPos) < POSITION_TOLERANCE) {
      // We've reached the target — flip direction
      m_movingUp = !m_movingUp;
      
      // Set new target (will be picked up in the next iteration)
      double newTarget = m_movingUp ? HIGH_POSITION : LOW_POSITION;
      m_intake.setTargetPosition(newTarget);
      
      // Only count a cycle completion when we reach LOW (bottom of wiggle)
      if (!m_movingUp) {
        m_completedCycles++;
      }
    } else {
      // Keep driving toward current target
      m_intake.setTargetPosition(targetPos);
    }
    
    // Dashboard telemetry
    SmartDashboard.putNumber("Wiggle/Current_Position", currentPos);
    SmartDashboard.putNumber("Wiggle/Target_Position", targetPos);
    SmartDashboard.putNumber("Wiggle/Low_Target", LOW_POSITION);
    SmartDashboard.putNumber("Wiggle/High_Target", HIGH_POSITION);
    SmartDashboard.putNumber("Wiggle/Cycle", m_completedCycles);
    SmartDashboard.putString("Wiggle/Status", 
        m_movingUp ? "Moving up to " + String.format("%.2f", HIGH_POSITION) 
                   : "Moving down to " + String.format("%.2f", LOW_POSITION));
  }

  @Override
  public void end(boolean interrupted) {
    // Return to fully deployed position
    m_intake.setTargetPosition(LOW_POSITION);
    
    SmartDashboard.putString("Wiggle/Status", 
        interrupted ? "Interrupted" : "Complete");
    System.out.println("WiggleIntakeCommand ended" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    // Command finishes when we've completed the requested number of cycles
    return m_completedCycles >= m_cycleCount;
  }
}
