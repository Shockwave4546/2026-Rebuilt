// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstantsProfiled;
import frc.robot.util.TelemetryRateLimiter;

/**
 * Alternate intake pivot subsystem that runs the motion profile entirely on the
 * roboRIO, eliminating the stutter caused by the REV MAXMotion firmware
 * re-planning its trajectory from a poorly-sampled encoder position.
 *
 * <p>Each 20 ms periodic loop the RIO:
 * <ol>
 *   <li>Advances a {@link TrapezoidProfile} by one time step to produce a smooth
 *       velocity-limited trajectory.</li>
 *   <li>Runs a {@link ProfiledPIDController} to correct position error against
 *       that profile's instantaneous setpoint.</li>
 *   <li>Adds an {@link ArmFeedforward} term (kS + kG·cos(θ) + kV·v) to
 *       pre-compensate for gravity and friction.</li>
 *   <li>Sends the combined duty-cycle value directly to the SparkMax via
 *       {@link SparkMax#set(double)}, bypassing the REV closed-loop firmware
 *       entirely.</li>
 * </ol>
 *
 * <p>Hardware soft limits on the SparkMax remain active as a mechanical safety
 * net even in open-loop mode.
 *
 * <p>This class has an identical public API to {@link IntakeSubsystem} so that
 * {@link frc.robot.commands.IntakePivotCommand} and {@link frc.robot.RobotContainer}
 * can swap between the two by changing a single instantiation.
 *
 * <p>Position units: rotations (absolute encoder, 0.0 – 1.0).
 * Velocity units: rotations/second.
 */
public class IntakeSubsystemProfiled extends SubsystemBase {

  private final SparkMax m_pivotMotor;
  private final SparkMax m_innerRollerMotor;  // CAN 31 — inner roller (closer to robot)
  private final SparkMax m_outerRollerMotor;  // CAN 32 — outer roller (ground-facing)

  private final AbsoluteEncoder m_pivotEncoder;

  /**
   * RIO-side trapezoidal profiled PID controller.
   * The goal is the desired final position; the controller internally tracks a
   * velocity-limited trajectory toward that goal each loop cycle.
   */
  private final ProfiledPIDController m_pivotPID;

  /**
   * Gravity + friction feedforward for a rotating arm.
   *
   * <p>Note: {@link ArmFeedforward} expects position in <em>radians</em> so its
   * kG·cos(θ) term is geometrically correct. We convert rotations → radians before
   * calling {@code calculate}.
   */
  private final ArmFeedforward m_pivotFF;

  /** Last clamped goal position, or {@code null} if the subsystem is idle. */
  private Double m_targetPosition = null;

  private boolean m_isPivotEnabled = false;
  private boolean m_isRollerRunning = false;
  private boolean m_isRollerReversing = false;

  // Unjam detection and recovery state
  private long m_stallDetectionStartTimeMs = 0;  // Time when low RPM + high current condition started
  private boolean m_isUnjamReversing = false;    // Currently executing reverse spin
  private long m_unjamReverseStartTimeMs = 0;    // Time when reverse spin started

  // Tunable roller speeds (read from SmartDashboard each cycle for live tuning during testing)
  private double m_innerRollerSpeed = IntakeConstants.kIntakeInnerRollerForwardSpeed;
  private double m_outerRollerSpeedMultiplier = IntakeConstants.kIntakeOuterRollerForwardSpeed;

  /** Rate limiter for telemetry updates (10Hz instead of 50Hz). */
  private final TelemetryRateLimiter m_telemetryRateLimiter;

  public IntakeSubsystemProfiled() {
    // --- Pivot motor ---
    m_pivotMotor = new SparkMax(IntakeConstants.kIntakePivotMotorCanId, MotorType.kBrushless);
    m_pivotMotor.configure(
        Configs.IntakePivotProfiled.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_pivotEncoder = m_pivotMotor.getAbsoluteEncoder();

    // ProfiledPIDController — units must match the encoder (rotations, rot/s).
    m_pivotPID = new ProfiledPIDController(
        IntakeConstantsProfiled.kP,
        IntakeConstantsProfiled.kI,
        IntakeConstantsProfiled.kD,
        new TrapezoidProfile.Constraints(
            IntakeConstantsProfiled.kMaxVelocity,
            IntakeConstantsProfiled.kMaxAcceleration));
    m_pivotPID.setTolerance(IntakeConstantsProfiled.kTolerance);

    // ArmFeedforward — kS: static friction, kG: gravity, kV: velocity, kA: accel.
    m_pivotFF = new ArmFeedforward(
        IntakeConstantsProfiled.kS,
        IntakeConstantsProfiled.kG,
        IntakeConstantsProfiled.kV,
        IntakeConstantsProfiled.kA);

    // Seed the controller with the current measured state so the very first move
    // starts from actual position/velocity — no step change at enable.
    resetPID();

    // --- Inner roller motor (CAN 31, NEO 550, open-loop only) ---
    m_innerRollerMotor = new SparkMax(IntakeConstants.kIntakeInnerRollerCanId, MotorType.kBrushless);
    SparkMaxConfig innerRollerConfig = new SparkMaxConfig();
    innerRollerConfig
        .inverted(IntakeConstants.kIntakeInnerRollerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kIntakeRollerCurrentLimit);
    m_innerRollerMotor.configure(innerRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Outer roller motor (CAN 32, NEO 550, open-loop only) ---
    // Faces opposite direction to inner roller, so inversion differs
    m_outerRollerMotor = new SparkMax(IntakeConstants.kIntakeOuterRollerCanId, MotorType.kBrushless);
    SparkMaxConfig outerRollerConfig = new SparkMaxConfig();
    outerRollerConfig
        .inverted(IntakeConstants.kIntakeOuterRollerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kIntakeRollerCurrentLimit);
    m_outerRollerMotor.configure(outerRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Rate limiter for telemetry (10Hz instead of 50Hz)
    m_telemetryRateLimiter = new TelemetryRateLimiter(10.0);

    // Seed dashboard with tunable roller speed controls (for live testing)
    SmartDashboard.putNumber("Intake/Tuning/Inner Roller Speed", IntakeConstants.kIntakeInnerRollerForwardSpeed);
    SmartDashboard.putNumber("Intake/Tuning/Outer Speed Multiplier", IntakeConstants.kIntakeOuterRollerForwardSpeed);
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // --- Read tunable roller speeds from dashboard (for live testing) ---
    m_innerRollerSpeed = SmartDashboard.getNumber("Intake/Tuning/Inner Roller Speed", IntakeConstants.kIntakeInnerRollerForwardSpeed);
    m_outerRollerSpeedMultiplier = SmartDashboard.getNumber("Intake/Tuning/Outer Speed Multiplier", IntakeConstants.kIntakeOuterRollerForwardSpeed);

    double currentPosition = getEncoderPosition();

    if (m_isPivotEnabled && m_targetPosition != null) {
      // Step the profile forward and compute PID correction.
      double pidOutput = m_pivotPID.calculate(currentPosition, m_targetPosition);

      // Feed the profile's instantaneous velocity setpoint into the feedforward so
      // the motor pre-compensates for the expected motion (not just reacting).
      TrapezoidProfile.State setpoint = m_pivotPID.getSetpoint();

      // Convert encoder rotations → physical angle in radians so ArmFeedforward's
      // cos(θ) term is geometrically correct across the full range of motion.
      //
      // Mapping: kGravityPeakPosition → 0 rad (arm horizontal, cos(0) = 1, max gravity torque)
      //          kGravityZeroPosition → π/2 rad (arm vertical, cos(π/2) = 0, no gravity torque)
      //          mirror position      → π rad (horizontal other side, cos(π) = −1, gravity reverses)
      //
      // Formula: θ = (pos − kGravityPeakPosition)
      //            / (kGravityZeroPosition − kGravityPeakPosition)
      //            * (π / 2)
      //   → at deployed (0.29): θ = 0 rad, cos(0) = 1 → full gravity
      //   → at weightless (0.58): θ = π/2 rad, cos(π/2) = 0 → zero gravity ✓
      double scale = IntakeConstantsProfiled.kGravityZeroPosition
                   - IntakeConstantsProfiled.kGravityPeakPosition;
      double posRad    = ((setpoint.position - IntakeConstantsProfiled.kGravityPeakPosition) / scale)
                       * (Math.PI / 2.0);
      double velRadPerSec = (setpoint.velocity / scale) * (Math.PI / 2.0);
      double ffOutput = m_pivotFF.calculate(posRad, velRadPerSec);

      double output = MathUtil.clamp(
          pidOutput + ffOutput,
          IntakeConstants.kIntakePivotMinOutput,
          IntakeConstants.kIntakePivotMaxOutput);

      // Duty-cycle directly to SparkMax — REV closed-loop firmware bypassed.
      m_pivotMotor.set(output);

      // --- Publish PID/FF debugging telemetry (rate-limited to 10Hz) ---
      if (m_telemetryRateLimiter.tryUpdate()) {
        SmartDashboard.putNumber("Intake/PID Output", pidOutput);
        SmartDashboard.putNumber("Intake/FF Output",  ffOutput);
        SmartDashboard.putNumber("Intake/Profile Pos (rot)", setpoint.position);
        SmartDashboard.putNumber("Intake/Profile Vel (rot-s)", setpoint.velocity);
      }
    } else {
      m_pivotMotor.set(0.0);
    }

    // --- Roller motor control with unjam detection/recovery ---
    updateRollerControl();


    // --- Dashboard telemetry (rate-limited to 10Hz, change-detection on continuous values) ---
    double velocity = m_pivotEncoder.getVelocity();
    double motorOutput = m_pivotMotor.get();
    double appliedVoltage = m_pivotMotor.getAppliedOutput() * 12.0;
    double current = m_pivotMotor.getOutputCurrent();
    boolean atTarget = isAtTarget();
    boolean deployed = isDeployed();
    boolean retracted = isRetracted();

    if (m_telemetryRateLimiter.hasChangedNumber("Intake/Position (rot)", currentPosition)) {
      SmartDashboard.putNumber("Intake/Position (rot)", currentPosition);
    }
    if (m_telemetryRateLimiter.hasChangedNumber("Intake/Velocity (rot-s)", velocity)) {
      SmartDashboard.putNumber("Intake/Velocity (rot-s)", velocity);
    }
    if (m_telemetryRateLimiter.hasChangedNumber("Intake/Motor Output", motorOutput)) {
      SmartDashboard.putNumber("Intake/Motor Output", motorOutput);
    }
    if (m_telemetryRateLimiter.hasChangedNumber("Intake/Applied Voltage (V)", appliedVoltage)) {
      SmartDashboard.putNumber("Intake/Applied Voltage (V)", appliedVoltage);
    }
    if (m_telemetryRateLimiter.hasChangedNumber("Intake/Current (A)", current)) {
      SmartDashboard.putNumber("Intake/Current (A)", current);
    }

    if (m_targetPosition != null) {
      if (m_telemetryRateLimiter.hasChangedNumber("Intake/Target (rot)", m_targetPosition)) {
        SmartDashboard.putNumber("Intake/Target (rot)", m_targetPosition);
      }
      double error = m_targetPosition - currentPosition;
      if (m_telemetryRateLimiter.hasChangedNumber("Intake/Error (rot)", error)) {
        SmartDashboard.putNumber("Intake/Error (rot)", error);
      }
    }

    if (m_telemetryRateLimiter.hasChangedBoolean("Intake/At Target", atTarget)) {
      SmartDashboard.putBoolean("Intake/At Target", atTarget);
    }
    if (m_telemetryRateLimiter.hasChangedBoolean("Intake/Deployed", deployed)) {
      SmartDashboard.putBoolean("Intake/Deployed", deployed);
    }
    if (m_telemetryRateLimiter.hasChangedBoolean("Intake/Retracted", retracted)) {
      SmartDashboard.putBoolean("Intake/Retracted", retracted);
    }
    if (m_telemetryRateLimiter.hasChangedBoolean("Intake/Pivot Enabled", m_isPivotEnabled)) {
      SmartDashboard.putBoolean("Intake/Pivot Enabled", m_isPivotEnabled);
    }
    if (m_telemetryRateLimiter.hasChangedBoolean("Intake/Roller Running", m_isRollerRunning)) {
      SmartDashboard.putBoolean("Intake/Roller Running", m_isRollerRunning);
    }

    // Always publish tuning controls (no rate limiting — need immediate updates)
    SmartDashboard.putNumber("Intake/Tuning/Inner Roller Speed", m_innerRollerSpeed);
    SmartDashboard.putNumber("Intake/Tuning/Outer Speed Multiplier", m_outerRollerSpeedMultiplier);
  }

  // ---------------------------------------------------------------------------
  // Roller control with unjam detection and recovery
  // ---------------------------------------------------------------------------

  /**
   * Updates roller motor output with automatic unjam detection and recovery.
   * 
   * <p>Logic flow:
   * <ol>
   *   <li>If currently reversing for unjam, check if reverse duration has elapsed.
   *       If so, return to normal intake direction and resume normal control.</li>
   *   <li>Otherwise, if rollers are commanded on and we detect low RPM (<1000)
   *       and high current (>18A) for 0.2s, trigger an unjam by reversing.</li>
   *   <li>Otherwise, apply normal roller control (forward/reverse/stop based on commands).</li>
   * </ol>
   */
  private void updateRollerControl() {
    long nowMs = System.currentTimeMillis();
    // Check the actual outer roller's (CAN 32) current draw for stall detection
    double outerCurrent = m_outerRollerMotor.getOutputCurrent();
    // Rollers allowed to run up through partial-deploy position (position-based, not just fully deployed)
    boolean rollersAllowed = getEncoderPosition() < IntakeConstants.kIntakeRollerMaxRunPosition;

    // If we're in the middle of an unjam reverse, check if it's time to stop.
    if (m_isUnjamReversing) {
      long elapsedMs = nowMs - m_unjamReverseStartTimeMs;
      if (elapsedMs >= (IntakeConstants.kUnjamReverseTimeS * 1000)) {
        // Unjam period complete; resume normal control
        m_isUnjamReversing = false;
        m_stallDetectionStartTimeMs = 0;
      } else {
        // Outer reverses to clear jam; inner keeps pushing forward to prevent ball backup
        m_outerRollerMotor.set(IntakeConstants.kIntakeUnjamOuterSpeed);
        m_innerRollerMotor.set(IntakeConstants.kIntakeUnjamInnerSpeed);
        if (m_telemetryRateLimiter.tryUpdate()) {
          SmartDashboard.putString("Intake/Unjam Status", "REVERSING");
        }
        return;
      }
    }

    // Detect jam condition: high current for sustained duration
    boolean isHighCurrent = outerCurrent > IntakeConstants.kUnjamCurrentThreshold;

    if (m_isRollerRunning && rollersAllowed && isHighCurrent) {
      // Jam condition detected; start or continue stall timer
      if (m_stallDetectionStartTimeMs == 0) {
        m_stallDetectionStartTimeMs = nowMs;
      }
      long stallDurationMs = nowMs - m_stallDetectionStartTimeMs;
      
      if (stallDurationMs >= (IntakeConstants.kUnjamDetectionTimeS * 1000)) {
        // Stall duration threshold met; initiate unjam
        m_isUnjamReversing = true;
        m_unjamReverseStartTimeMs = nowMs;
        m_stallDetectionStartTimeMs = 0;
        if (m_telemetryRateLimiter.tryUpdate()) {
          SmartDashboard.putString("Intake/Unjam Status", "JAM DETECTED");
        }
      }
    } else {
      // No jam condition; reset stall timer
      m_stallDetectionStartTimeMs = 0;
    }

    // Normal roller control (only applies if not in unjam mode)
    if (m_isRollerReversing && rollersAllowed) {
      m_innerRollerMotor.set(-m_innerRollerSpeed);  // CAN 31 full speed reverse
      m_outerRollerMotor.set(-m_innerRollerSpeed * m_outerRollerSpeedMultiplier);  // CAN 32 reduced speed reverse
      if (m_telemetryRateLimiter.tryUpdate()) {
        SmartDashboard.putString("Intake/Unjam Status", "REVERSING_MANUAL");
      }
    } else if (m_isRollerRunning && rollersAllowed) {
      m_innerRollerMotor.set(m_innerRollerSpeed);  // CAN 31 full speed forward
      m_outerRollerMotor.set(m_innerRollerSpeed * m_outerRollerSpeedMultiplier);  // CAN 32 reduced speed forward
      if (m_telemetryRateLimiter.tryUpdate()) {
        SmartDashboard.putString("Intake/Unjam Status", "RUNNING");
      }
    } else {
      m_innerRollerMotor.set(0.0);
      m_outerRollerMotor.set(0.0);
      if (m_telemetryRateLimiter.tryUpdate()) {
        SmartDashboard.putString("Intake/Unjam Status", "STOPPED");
      }
    }

    // Roller diagnostics (no rate limiting — real-time visibility for speed tuning)
    SmartDashboard.putNumber("Intake/Inner RPM",         m_innerRollerMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake/Inner Current A",   m_innerRollerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Inner Output",      m_innerRollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Outer RPM",         m_outerRollerMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake/Outer Current A",   m_outerRollerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Outer Output",      m_outerRollerMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Intake/High Current",     isHighCurrent);
  }
  /**
   * Command the pivot to a target position.
   *
   * <p>The RIO's {@link ProfiledPIDController} will generate a smooth trapezoidal
   * trajectory from the current position/velocity to the goal, updating every 20 ms.
   *
   * @param targetPosition Desired position in rotations (clamped to soft limits).
   */
  public void setTargetPosition(double targetPosition) {
    double clamped = MathUtil.clamp(
        targetPosition,
        IntakeConstants.kIntakePivotMinPosition,
        IntakeConstants.kIntakePivotMaxPosition);

    // Seed the profile from current state only when coming out of idle, so that
    // mid-move goal changes continue tracking rather than restarting the profile.
    if (!m_isPivotEnabled) {
      resetPID();
    }

    m_targetPosition = clamped;
    m_isPivotEnabled = true;
  }

  /**
   * Stop all motion and disable the pivot controller.
   * The pivot motor output goes to 0; the arm may drift under gravity until
   * {@link #setTargetPosition} is called again.
   */
  public void stop() {
    m_isPivotEnabled  = false;
    m_targetPosition  = null;
    m_isRollerRunning = false;
    m_pivotMotor.set(0.0);
    m_innerRollerMotor.set(0.0);
    m_outerRollerMotor.set(0.0);
  }

  /** Enable the intake rollers (they will only spin while the arm is deployed). */
  public void run() {
    m_isRollerRunning = true;
    m_isRollerReversing = false;
  }

  /** Stop the intake rollers without affecting pivot control. */
  public void stopRollers() {
    m_isRollerRunning = false;
    m_isRollerReversing = false;
  }

  /** Reverse the intake rollers to eject game pieces. */
  public void reverseRollers() {
    m_isRollerReversing = true;
    m_isRollerRunning = false;
  }

  // ---------------------------------------------------------------------------
  // State queries
  // ---------------------------------------------------------------------------

  /** Current absolute encoder position, clamped to [0, 1] rotations. */
  public double getEncoderPosition() {
    return MathUtil.clamp(m_pivotEncoder.getPosition(), 0.0, 1.0);
  }

  /**
   * {@code true} when the pivot has reached the goal position within
   * {@link IntakeConstantsProfiled#kTolerance} and the profile's velocity
   * setpoint is also at zero (i.e. {@link ProfiledPIDController#atGoal()}).
   */
  public boolean isAtTarget() {
    if (!m_isPivotEnabled || m_targetPosition == null) {
      return false;
    }
    return m_pivotPID.atGoal();
  }

  /** {@code true} when encoder position is below the deployed threshold (arm is down). */
  public boolean isDeployed() {
    return getEncoderPosition() < IntakeConstants.kIntakePivotDeployedThreshold;
  }

  /** {@code true} when encoder position is above the retracted threshold (arm is up). */
  public boolean isRetracted() {
    return getEncoderPosition() > IntakeConstants.kIntakePivotRetractedThreshold;
  }

  /** The last goal position, or {@code null} if no command has been issued. */
  public Double getTargetPosition() {
    return m_targetPosition;
  }

  /** Whether the pivot controller is currently active. */
  public boolean isPivotEnabled() {
    return m_isPivotEnabled;
  }

  /** Whether the roller motor has been commanded on. */
  public boolean areRollersRunning() {
    return m_isRollerRunning;
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  /**
   * Seeds the {@link ProfiledPIDController} with the current measured position and
   * velocity so the first control output after enable is bumpless.
   */
  private void resetPID() {
    m_pivotPID.reset(
        getEncoderPosition(),
        m_pivotEncoder.getVelocity());
  }
}
