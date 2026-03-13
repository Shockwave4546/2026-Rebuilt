// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

/**
 * Indexer subsystem for feeding game pieces.
 *
 * <p>Simple open-loop voltage control (12V max) for a single indexer motor
 * on CAN ID 40 (NEO 550).
 */
public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax m_indexerMotor;

    private boolean m_isRunning = false;
    private boolean m_isReversing = false;

    public IndexerSubsystem() {
        m_indexerMotor = new SparkMax(IndexerConstants.kIndexerMotorCanId, MotorType.kBrushless);
        m_indexerMotor.configure(
                Configs.Indexer.indexerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        if (m_isRunning) {
            m_indexerMotor.setVoltage(IndexerConstants.kIndexerVoltage);
        } else if (m_isReversing) {
            m_indexerMotor.setVoltage(-IndexerConstants.kIndexerVoltage);
        } else {
            m_indexerMotor.stopMotor();
        }

        // Dashboard telemetry
        SmartDashboard.putBoolean("Indexer/Running", m_isRunning);
        SmartDashboard.putBoolean("Indexer/Reversing", m_isReversing);
        SmartDashboard.putNumber("Indexer/Applied Voltage (V)", 
                m_indexerMotor.getAppliedOutput() * 12.0);
        SmartDashboard.putNumber("Indexer/Current (A)", m_indexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Indexer/Motor Output %", m_indexerMotor.getAppliedOutput() * 100.0);
    }

    /**
     * Run the indexer at full voltage (12V).
     */
    public void run() {
        m_isRunning = true;
        m_isReversing = false;
    }

    /**
     * Run the indexer in reverse at full voltage (12V).
     */
    public void runReverse() {
        m_isRunning = false;
        m_isReversing = true;
    }

    /**
     * Stop the indexer (motor coasts to rest).
     */
    public void stop() {
        m_isRunning = false;
        m_isReversing = false;
    }

    /**
     * @return {@code true} if the indexer is currently running.
     */
    public boolean isRunning() {
        return m_isRunning;
    }
}
