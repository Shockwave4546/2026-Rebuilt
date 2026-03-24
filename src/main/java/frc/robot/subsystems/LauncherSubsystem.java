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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.WiggleIntakeCommand;
import frc.robot.subsystems.IntakeSubsystemProfiled;
import frc.robot.util.TelemetryRateLimiter;

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

    /** Whether the feeder is running in reverse independently. */
    private boolean m_isFeederReversing = false;

    /** Target RPM read from SmartDashboard each loop. */
    private double m_targetRpm = LauncherConstants.kShooterTargetRpm;

    /** Adjustable RPM for short-distance shots (tuned during matches). */
    private double m_shortRpm = LauncherConstants.kShooterShortRpm;

    /** Adjustable RPM for long-distance shots (tuned during matches). */
    private double m_longRpm = LauncherConstants.kShooterLongRpm;

    /** Reference to indexer subsystem for automatic feeding when at target RPM. */
    private IndexerSubsystem m_indexer = null;

    /** Optional reference to intake subsystem so we can auto-wiggle while shooting. */
    private IntakeSubsystemProfiled m_intake = null;

    /** Wiggle command instance when scheduled by the launcher (long-running while shooting). */
    private Command m_wiggleCommand = null;

    /** Rate limiter for telemetry updates (10Hz instead of 50Hz). */
    private final TelemetryRateLimiter m_telemetryRateLimiter = new TelemetryRateLimiter(10.0);

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

                // If intake is present, schedule a long-running wiggle command while shooting
                if (m_intake != null) {
                    // If we don't have one scheduled, create and schedule it
                    if (m_wiggleCommand == null || !CommandScheduler.getInstance().isScheduled(m_wiggleCommand)) {
                        m_wiggleCommand = new WiggleIntakeCommand(m_intake, Integer.MAX_VALUE);
                        CommandScheduler.getInstance().schedule(m_wiggleCommand);
                    }
                }
            } else {
                m_feederMotor.stopMotor();

                // Shooter not at RPM — cancel any wiggle scheduled by the launcher
                if (m_wiggleCommand != null) {
                    CommandScheduler.getInstance().cancel(m_wiggleCommand);
                    m_wiggleCommand = null;
                }
            }

            //TODO
            //Auto unjam workflow-
            //Current based -
            //If feeder or indexer stalled for >1s
            //pause .5 reverse .75?
            //Time based - 
            //Every 3s of shooting 
            //.5s stop 1s reverse
            
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
        } else if (m_isFeederReversing) {
            // Feeder reverse only — shooter stays off
            m_feederMotor.setVoltage(-LauncherConstants.kFeederVoltage);
            m_shooterLeader.stopMotor();
        } else {
            // stopMotor() puts the controller into idle — respects IdleMode.kCoast
            m_feederMotor.stopMotor();
            m_shooterLeader.stopMotor();
            m_shooterPID.reset();
            // Follower coasts automatically when leader is idle
        }

        // --- Dashboard telemetry (rate-limited to 10Hz, change-detection on continuous values) ---
        double ffVolts = m_shooterFF.calculate(m_targetRpm);
        double shooterRpm = m_shooterEncoder.getVelocity();
        double appliedOutput = m_shooterLeader.getAppliedOutput();
        double shooterCurrent = m_shooterLeader.getOutputCurrent();
        double feederCurrent = m_feederMotor.getOutputCurrent();
        boolean atTargetRpm = isAtTargetRpm();

        if (m_telemetryRateLimiter.hasChangedNumber("Launcher/Shooter RPM", shooterRpm)) {
            SmartDashboard.putNumber("Launcher/Shooter RPM", shooterRpm);
        }
        if (m_telemetryRateLimiter.hasChangedBoolean("Launcher/At Target RPM", atTargetRpm)) {
            SmartDashboard.putBoolean("Launcher/At Target RPM", atTargetRpm);
        }
        if (m_telemetryRateLimiter.hasChangedNumber("Launcher/FF Voltage (V)", ffVolts)) {
            SmartDashboard.putNumber("Launcher/FF Voltage (V)", ffVolts);
        }
        if (m_telemetryRateLimiter.hasChangedNumber("Launcher/Applied Output", appliedOutput)) {
            SmartDashboard.putNumber("Launcher/Applied Output", appliedOutput);
        }
        if (m_telemetryRateLimiter.hasChangedNumber("Launcher/Shooter Current (A)", shooterCurrent)) {
            SmartDashboard.putNumber("Launcher/Shooter Current (A)", shooterCurrent);
        }
        if (m_telemetryRateLimiter.hasChangedNumber("Launcher/Feeder Current (A)", feederCurrent)) {
            SmartDashboard.putNumber("Launcher/Feeder Current (A)", feederCurrent);
        }
        if (m_telemetryRateLimiter.hasChangedBoolean("Launcher/Running", m_isRunning)) {
            SmartDashboard.putBoolean("Launcher/Running", m_isRunning);
        }
        if (m_telemetryRateLimiter.hasChangedBoolean("Launcher/Spinning Up", m_isSpinningUp)) {
            SmartDashboard.putBoolean("Launcher/Spinning Up", m_isSpinningUp);
        }
        
        // Debug: indexer status
        boolean indexerConnected = m_indexer != null;
        if (m_telemetryRateLimiter.hasChangedBoolean("Launcher/Indexer Connected", indexerConnected)) {
            SmartDashboard.putBoolean("Launcher/Indexer Connected", indexerConnected);
        }
        if (m_indexer != null) {
            if (m_telemetryRateLimiter.hasChangedBoolean("Launcher/Indexer Running (from Launcher)", m_indexer.isRunning())) {
                SmartDashboard.putBoolean("Launcher/Indexer Running (from Launcher)", m_indexer.isRunning());
            }
            boolean shouldStartIndexer = m_isRunning && isAtTargetRpm();
            if (m_telemetryRateLimiter.hasChangedBoolean("Launcher/Should Start Indexer", shouldStartIndexer)) {
                SmartDashboard.putBoolean("Launcher/Should Start Indexer", shouldStartIndexer);
            }
            // Wiggle scheduled status
            boolean wiggleScheduled = (m_wiggleCommand != null) && CommandScheduler.getInstance().isScheduled(m_wiggleCommand);
            if (m_telemetryRateLimiter.hasChangedBoolean("Launcher/Wiggle Scheduled", wiggleScheduled)) {
                SmartDashboard.putBoolean("Launcher/Wiggle Scheduled", wiggleScheduled);
            }
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
        m_isFeederReversing = false;
        if (m_indexer != null) m_indexer.stop();
        // Cancel any wiggle that the launcher scheduled
        if (m_wiggleCommand != null) {
            CommandScheduler.getInstance().cancel(m_wiggleCommand);
            m_wiggleCommand = null;
        }
    }

    /**
     * Run only the feeder motor independently (no shooter).
     */
    public void runFeeder() {
        m_isRunning = false;
        m_isSpinningUp = false;
        m_isFeederReversing = false;
        m_isFeederRunning = true;
    }

    /**
     * Run the feeder in reverse independently (no shooter). Useful for unjamming.
     */
    public void runFeederReverse() {
        m_isRunning = false;
        m_isSpinningUp = false;
        m_isFeederRunning = false;
        m_isFeederReversing = true;
    }

    /**
     * Stop the feeder when running independently.
     */
    public void stopFeeder() {
        m_isFeederRunning = false;
        m_isFeederReversing = false;
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
     * Sets a reference to the intake subsystem so the launcher can schedule
     * the wiggle command while shooting. Call this from RobotContainer during setup.
     */
    public void setIntake(IntakeSubsystemProfiled intake) {
        m_intake = intake;
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
