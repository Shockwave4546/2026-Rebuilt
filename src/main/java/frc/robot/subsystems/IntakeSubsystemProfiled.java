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
  private final SparkMax m_rollerMotor;

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

    // --- Roller motor (NEO 550, open-loop only) ---
    m_rollerMotor = new SparkMax(IntakeConstants.kIntakeRollerMotorCanId, MotorType.kBrushless);
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .inverted(IntakeConstants.kIntakeRollerMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kIntakeRollerCurrentLimit);
    m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
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
      // Mapping: kGravityPeakPosition → 0 rad (arm horizontal, cos = 1, max gravity torque)
      //          kGravityZeroPosition → π/2 rad (arm vertical, cos = 0, no gravity torque)
      //          mirror position      → π rad (horizontal other side, cos = −1, gravity reverses)
      //
      // Formula: θ = (pos − kGravityZeroPosition)
      //            / (kGravityZeroPosition − kGravityPeakPosition)
      //            * (π / 2)
      //   → shifted so vertical = π/2, then scaled to the arm's travel range.
      double scale = IntakeConstantsProfiled.kGravityZeroPosition
                   - IntakeConstantsProfiled.kGravityPeakPosition;
      double posRad    = ((setpoint.position - IntakeConstantsProfiled.kGravityZeroPosition) / scale)
                       * (Math.PI / 2.0);
      double velRadPerSec = (setpoint.velocity / scale) * (Math.PI / 2.0);
      double ffOutput = m_pivotFF.calculate(posRad, velRadPerSec);

      double output = MathUtil.clamp(
          pidOutput + ffOutput,
          IntakeConstants.kIntakePivotMinOutput,
          IntakeConstants.kIntakePivotMaxOutput);

      // Duty-cycle directly to SparkMax — REV closed-loop firmware bypassed.
      m_pivotMotor.set(output);

      SmartDashboard.putNumber("Intake/PID Output", pidOutput);
      SmartDashboard.putNumber("Intake/FF Output",  ffOutput);
      SmartDashboard.putNumber("Intake/Profile Pos (rot)", setpoint.position);
      SmartDashboard.putNumber("Intake/Profile Vel (rot-s)", setpoint.velocity);
    } else {
      m_pivotMotor.set(0.0);
    }

    // Rollers only spin while the arm is deployed.
    if (m_isRollerReversing && isDeployed()) {
      m_rollerMotor.set(-IntakeConstants.kIntakeRollerSpeed);  // Reverse
    } else if (m_isRollerRunning && isDeployed()) {
      m_rollerMotor.set(IntakeConstants.kIntakeRollerSpeed);   // Forward
    } else {
      m_rollerMotor.set(0.0);
    }

    // --- Dashboard telemetry ---
    SmartDashboard.putNumber("Intake/Position (rot)", currentPosition);
    SmartDashboard.putNumber("Intake/Velocity (rot-s)", m_pivotEncoder.getVelocity());
    SmartDashboard.putNumber("Intake/Motor Output", m_pivotMotor.get());
    SmartDashboard.putNumber("Intake/Applied Voltage (V)", m_pivotMotor.getAppliedOutput() * 12.0);
    SmartDashboard.putNumber("Intake/Current (A)", m_pivotMotor.getOutputCurrent());

    if (m_targetPosition != null) {
      SmartDashboard.putNumber("Intake/Target (rot)", m_targetPosition);
      SmartDashboard.putNumber("Intake/Error (rot)", m_targetPosition - currentPosition);
    }

    SmartDashboard.putBoolean("Intake/At Target",      isAtTarget());
    SmartDashboard.putBoolean("Intake/Deployed",       isDeployed());
    SmartDashboard.putBoolean("Intake/Retracted",      isRetracted());
    SmartDashboard.putBoolean("Intake/Pivot Enabled",  m_isPivotEnabled);
    SmartDashboard.putBoolean("Intake/Roller Running", m_isRollerRunning);
  }

  // ---------------------------------------------------------------------------
  // Public API
  // ---------------------------------------------------------------------------

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
    m_rollerMotor.set(0.0);
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
