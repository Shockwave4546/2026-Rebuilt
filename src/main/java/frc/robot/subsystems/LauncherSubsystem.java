// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.LauncherConstants;

/**
 * Launcher subsystem consisting of:
 * <ul>
 *   <li><b>Feeder</b> — NEO 550 on SparkMax (CAN 50), open-loop voltage control.
 *       Spins at a fixed voltage to feed game pieces into the shooter wheels.</li>
 *   <li><b>Shooter Leader</b> — Vortex on SparkFlex (CAN 51), velocity PID on the
 *       built-in encoder to maintain constant RPM while game pieces pass through.</li>
 *   <li><b>Shooter Follower</b> — Vortex on SparkFlex (CAN 52), configured as a
 *       hardware follower of the leader with output inversion (same shaft,
 *       physically reversed motor).</li>
 * </ul>
 *
 * <p>Call {@link #runLauncher()} / {@link #stopLauncher()} to start and stop both
 * the feeder and shooter together.  Use {@link #isAtTargetRpm()} to gate game-piece
 * release until the shooter has spooled up.
 */
public class LauncherSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax  m_feederMotor;
    private final SparkFlex m_shooterLeader;
    private final SparkFlex m_shooterFollower;

    // Shooter encoder (leader only — follower is hardware-slaved)
    private final RelativeEncoder m_shooterEncoder;

    // RoboRIO-side control loop — bypasses REV firmware velocity loop entirely
    // so the SparkFlex never enters its "at setpoint" idle/violet state.
    private final SimpleMotorFeedforward m_shooterFF;
    private final PIDController          m_shooterPID;

    /** Whether the launcher is currently commanded to run (feeder + shooter). */
    private boolean m_isRunning = false;

    /** Whether only the shooter wheels are spinning (no feeder). */
    private boolean m_isSpinningUp = false;

    /** Whether the feeder is running independently (no shooter). */
    private boolean m_isFeederRunning = false;

    /** Target RPM read from SmartDashboard each loop. */
    private double m_targetRpm = LauncherConstants.kShooterTargetRpm;

    /** Adjustable RPM for short-distance shots (tuned during matches). */
    private double m_shortRpm = LauncherConstants.kShooterShortRpm;

    /** Adjustable RPM for long-distance shots (tuned during matches). */
    private double m_longRpm = LauncherConstants.kShooterLongRpm;

    /** Reference to indexer subsystem for automatic feeding when at target RPM. */
    private IndexerSubsystem m_indexer = null;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public LauncherSubsystem() {
        // --- Feeder (NEO 550 / SparkMax) ---
        m_feederMotor = new SparkMax(LauncherConstants.kFeederMotorCanId, MotorType.kBrushless);
        m_feederMotor.configure(
                Configs.LauncherFeeder.feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // --- Shooter leader (Vortex / SparkFlex) ---
        m_shooterLeader = new SparkFlex(LauncherConstants.kShooterLeaderCanId, MotorType.kBrushless);
        m_shooterLeader.configure(
                Configs.LauncherShooter.leaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_shooterEncoder    = m_shooterLeader.getEncoder();

        // WPILib-side control loop — SparkFlex receives plain voltage via setVoltage(),
        // so the REV firmware never runs its own velocity loop and can't idle at setpoint.
        m_shooterFF  = new SimpleMotorFeedforward(
                LauncherConstants.kS_Shooter,
                LauncherConstants.kV_Shooter,
                LauncherConstants.kA_Shooter);
        m_shooterPID = new PIDController(
                LauncherConstants.kP_Shooter,
                LauncherConstants.kI_Shooter,
                LauncherConstants.kD_Shooter);

        // --- Shooter follower (Vortex / SparkFlex, inverted) ---
        m_shooterFollower = new SparkFlex(LauncherConstants.kShooterFollowerCanId, MotorType.kBrushless);
        m_shooterFollower.configure(
                Configs.LauncherShooter.followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Seed the dashboard entries so the fields are visible immediately on boot
        SmartDashboard.putNumber("Launcher/Short Shot RPM", LauncherConstants.kShooterShortRpm);
        SmartDashboard.putNumber("Launcher/Long Shot RPM",  LauncherConstants.kShooterLongRpm);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Read adjustable RPM values from dashboard every loop so driver changes take effect immediately
        m_shortRpm = SmartDashboard.getNumber("Launcher/Short Shot RPM", LauncherConstants.kShooterShortRpm);
        m_longRpm  = SmartDashboard.getNumber("Launcher/Long Shot RPM",  LauncherConstants.kShooterLongRpm);

        if (m_isRunning) {
            // Shooter: always spins up when commanded
            double voltage = m_shooterFF.calculate(m_targetRpm)
                           + m_shooterPID.calculate(m_shooterEncoder.getVelocity(), m_targetRpm);
            m_shooterLeader.setVoltage(voltage);

            // Feeder (50) and indexer (40) only engage once shooter is at target RPM
            if (isAtTargetRpm()) {
                m_feederMotor.setVoltage(LauncherConstants.kFeederVoltage);
                if (m_indexer != null) m_indexer.run();
            } else {
                m_feederMotor.stopMotor();
            }
        } else if (m_isSpinningUp) {
            // Shooter only — feeder idles/coasts
            m_feederMotor.stopMotor();
            double voltage = m_shooterFF.calculate(m_targetRpm)
                           + m_shooterPID.calculate(m_shooterEncoder.getVelocity(), m_targetRpm);
            m_shooterLeader.setVoltage(voltage);
        } else if (m_isFeederRunning) {
            // Feeder only — shooter stays off
            m_feederMotor.setVoltage(LauncherConstants.kFeederVoltage);
            m_shooterLeader.stopMotor();
        } else {
            // stopMotor() puts the controller into idle — respects IdleMode.kCoast
            m_feederMotor.stopMotor();
            m_shooterLeader.stopMotor();
            m_shooterPID.reset();
            // Follower coasts automatically when leader is idle
        }

        // --- Dashboard telemetry ---
        double ffVolts = m_shooterFF.calculate(m_targetRpm);
        SmartDashboard.putNumber("Launcher/Shooter RPM",         m_shooterEncoder.getVelocity());
        SmartDashboard.putBoolean("Launcher/At Target RPM",      isAtTargetRpm());
        SmartDashboard.putNumber("Launcher/FF Voltage (V)",      ffVolts);
        SmartDashboard.putNumber("Launcher/Applied Output",      m_shooterLeader.getAppliedOutput());
        SmartDashboard.putNumber("Launcher/Shooter Current (A)", m_shooterLeader.getOutputCurrent());
        SmartDashboard.putNumber("Launcher/Feeder Current (A)",  m_feederMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Launcher/Running",            m_isRunning);
        SmartDashboard.putBoolean("Launcher/Spinning Up",        m_isSpinningUp);
        
        // Debug: indexer status
        SmartDashboard.putBoolean("Launcher/Indexer Connected", m_indexer != null);
        if (m_indexer != null) {
            SmartDashboard.putBoolean("Launcher/Indexer Running (from Launcher)", m_indexer.isRunning());
            SmartDashboard.putBoolean("Launcher/Should Start Indexer", m_isRunning && isAtTargetRpm());
        }
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Start the feeder and spool up the shooter to the target RPM.
     */
    public void runLauncher() {
        m_isRunning = true;
        m_isSpinningUp = false;
        m_isFeederRunning = false;
    }

    /**
     * Stop the feeder and shooter (motors coast to rest).
     */
    public void stopLauncher() {
        m_isRunning = false;
        m_isSpinningUp = false;
        m_isFeederRunning = false;
        if (m_indexer != null) m_indexer.stop();
    }

    /**
     * Run only the feeder motor independently (no shooter).
     */
    public void runFeeder() {
        m_isRunning = false;
        m_isSpinningUp = false;
        m_isFeederRunning = true;
    }

    /**
     * Stop the feeder when running independently.
     */
    public void stopFeeder() {
        m_isFeederRunning = false;
    }

    /**
     * Run only the shooter wheels (useful for pre-spinning before feeding).
     * Feeder stays off until {@link #runLauncher()} is called.
     */
    public void spinUpShooter() {
        m_isRunning = false;
        m_isSpinningUp = true;
        m_isFeederRunning = false;
    }

    /**
     * @return {@code true} when the shooter is within {@link LauncherConstants#kShooterRpmTolerance}
     *         of the target RPM.
     */
    public boolean isAtTargetRpm() {
        return Math.abs(m_shooterEncoder.getVelocity() - m_targetRpm)
                <= LauncherConstants.kShooterRpmTolerance;
    }

    /**
     * @return Current shooter speed in RPM.
     */
    public double getShooterRpm() {
        return m_shooterEncoder.getVelocity();
    }

    /**
     * @return {@code true} if the launcher is commanded to run.
     */
    public boolean isRunning() {
        return m_isRunning;
    }

    /**
     * Sets a reference to the indexer subsystem for automatic feeding.
     * Call this from RobotContainer during setup.
     */
    public void setIndexer(IndexerSubsystem indexer) {
        m_indexer = indexer;
    }

    /**
     * Set the shooter to fire a short-distance shot using the adjustable short-range RPM.
     * Spins up the shooter and feeds the game piece.
     */
    public void shootShort() {
        m_targetRpm = m_shortRpm;
        runLauncher();
    }

    /**
     * Set the shooter to fire a long-distance shot using the adjustable long-range RPM.
     * Spins up the shooter and feeds the game piece.
     */
    public void shootLong() {
        m_targetRpm = m_longRpm;
        runLauncher();
    }

    /**
     * Spin up the shooter only for a short-distance shot (no feeder).
     * Allows pre-spinning before releasing the game piece.
     */
    public void spinUpShort() {
        m_targetRpm = m_shortRpm;
        spinUpShooter();
    }

    /**
     * Spin up the shooter only for a long-distance shot (no feeder).
     * Allows pre-spinning before releasing the game piece.
     */
    public void spinUpLong() {
        m_targetRpm = m_longRpm;
        spinUpShooter();
    }
}
