// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 10;
    public static final int kRearRightDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontLeftDrivingCanId = 13;

    public static final int kFrontRightTurningCanId = 20;
    public static final int kRearRightTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontLeftTurningCanId = 23;

    // Pigeon2 Gyro CAN ID
    public static final int kGyroCanId = 30;

    // Pigeon2 update frequency in Hz (default 100 Hz from factory)
    // Higher = more frequent updates but more CAN bus load
    // 100 Hz is standard for swerve odometry
    public static final double kGyroUpdateFrequencyHz = 100.0;

    public static final boolean kGyroReversed = false;

    // Thrust expo for smooth stick control
    // Higher values = more responsive at low stick inputs
    // Formula: output = (input^3) * (1 - kThrustExpo) + input * kThrustExpo
    public static final double kThrustExpo = 3;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Motor current limits (in Amps)
    public static final int kDrivingMotorCurrentLimit = 33;
    public static final int kTurningMotorCurrentLimit = 20;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class HeadingControllerConstants {
    // PID gains for heading control
    public static final double kHeadingP = 0.0048;//.00015 is close
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.00008;
    
    // Feed-forward gain for heading control
    // Helps overcome friction and provides faster response
    public static final double kHeadingFF = 0.006;//.006 small oscillations
    
    // Maximum rotation speed for heading controller (0.0 to 1.0)
    public static final double kMaxHeadingRotationSpeed = 10.0;
    
    // Tolerance for heading in degrees
    public static final double kHeadingTolerance = 2.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionAlignmentConstants {
    // Target distances for alignment
    public static final double kTargetDistanceX = 1.5; // meters - distance in front of tag
    public static final double kTargetOffsetY = 0.0; // meters - Y offset from tag center

    // PID gains for vision-based alignment
    // X distance control (moving toward/away from tag)
    public static final double kVisionXP = 0.05;
    public static final double kVisionXI = 0.0;
    public static final double kVisionXD = 0.001;

    // Y position control (side-to-side alignment)
    public static final double kVisionYP = 0.05;
    public static final double kVisionYI = 0.0;
    public static final double kVisionYD = 0.001;

    // Rotation/Yaw control (facing the tag)
    public static final double kVisionYawP = 0.001;
    public static final double kVisionYawI = 0.0;
    public static final double kVisionYawD = 0.0001;

    // Tolerance values for alignment completion
    public static final double kXTolerance = 0.03; // meters
    public static final double kYTolerance = 0.03; // meters
    public static final double kYawTolerance = 2.0; // degrees
  }

  public static final class IntakeConstants {
    // Motor CAN IDs
    public static final int kIntakePivotMotorCanId = 30;
    public static final int kIntakeRollerMotorCanId = 31;
    public static final int kIntakeRollerFollowerMotorCanId = 32;

    // Motor inversion
    public static final boolean kIntakePivotMotorInverted = true;
    public static final boolean kIntakeRollerMotorInverted = true;
    // Follower is inverted relative to leader (mounted facing opposite direction)
    public static final boolean kIntakeRollerFollowerMotorInverted = false;

    // Encoder configuration (REV Through-Bore Encoder V2 on SparkMax)
    public static final boolean kIntakePivotEncoderInverted = false;

    // Position limits (in rotations from the absolute encoder)
    // Deployed (down) ≈ 0.079, No gravity ≈ 0.23, Retracted (up) ≈ 0.292
    // Encoder increases as arm rises; min = deployed, max = retracted
    public static final double kIntakePivotMinPosition = 0.05;   // rotations (deployed, hard limit)
    public static final double kIntakePivotMaxPosition = 0.35;   // rotations (retracted, hard limit)

    // Named setpoints
    public static final double kIntakePivotDeployedPosition  = 0.079; // rotations
    public static final double kIntakePivotWeightlessPosition = 0.23; // rotations
    public static final double kIntakePivotRetractedPosition = 0.292; // rotations

    // Threshold for deployed/retracted state detection (rotations)
    public static final double kIntakePivotDeployedThreshold  = kIntakePivotDeployedPosition + 0.05;
    public static final double kIntakePivotRetractedThreshold = kIntakePivotRetractedPosition - 0.05;

    // MAXMotion PID gains (slot 0, encoder units = rotations)
    // kP: volts per rotation of error; kD dampen oscillations at setpoint
    public static final double kP_IntakePivot = 18.0;  // reduced from 20 to eliminate stutter
    public static final double kI_IntakePivot = 0.0;
    public static final double kD_IntakePivot = 1;   // dampen P oscillations at target
    public static final double kFF_IntakePivot = 0.0; // minimal FF; gravity cancels out between setpoints

    // MAXMotion profile constraints
    // Velocity in rotations/second, acceleration in rotations/second²
    public static final double kIntakePivotMaxVelocity     = 60;  // fast moves
    public static final double kIntakePivotMaxAcceleration = 40;  // snappy accel

    // Absolute encoder signal update period (ms). Lower = more frequent feedback to the
    // onboard MAXMotion controller. Default is 20ms; 10ms doubles the update rate.
    public static final int kIntakePivotEncoderPositionPeriodMs = 20;
    public static final int kIntakePivotEncoderVelocityPeriodMs = 20;

    // Allowed closed-loop error for isAtTarget() — must match or be looser than
    // the MAXMotion allowedProfileError set on the controller.
    // Looser value = fewer profile recomputation cycles when oscillating near setpoint.
    public static final double kIntakePivotTolerance = 0.017;       // rotations (~7.2 degrees)

    // Output range (fraction of bus voltage)
    public static final double kIntakePivotMinOutput = -1.0;
    public static final double kIntakePivotMaxOutput =  1.0;

    // Current limits (Amps)
    public static final int kIntakePivotCurrentLimit  = 40;  // NEO on pivot
    public static final int kIntakeRollerCurrentLimit = 20;  // NEO 550 on roller

    // Roller speed (duty cycle, 0-1)
    public static final double kIntakeRollerSpeed = 1.0;
  }

  /**
   * Constants for the RIO-side profiled intake pivot controller
   * ({@link frc.robot.subsystems.IntakeSubsystemProfiled}).
   *
   * <p>Physical setpoints, limits, CAN IDs, and encoder config are shared with
   * {@link IntakeConstants}. Only the control-law gains differ.
   *
   * <p>The RIO runs a WPILib {@code TrapezoidProfile} + {@code ProfiledPIDController}
   * every 20 ms and sends a plain duty-cycle command to the SparkMax, bypassing the
   * REV MAXMotion firmware entirely.
   *
   * <p>Units: position = rotations, velocity = rotations/second.
   */
  public static final class IntakeConstantsProfiled {

    // TrapezoidProfile constraints — match the old MAXMotion values so the arm
    // moves at the same speed.  Tune here to change feel.
    public static final double kMaxVelocity     = 6*0.0278; // rot/s (10 degrees/sec)
    public static final double kMaxAcceleration = 6*0.0278; // rot/s² (matching velocity for smooth ramp)

    // ProfiledPIDController gains (output = duty cycle, −1.0 to 1.0).
    // kP: duty-cycle per rotation of position error.
    // kD: damps velocity overshoot near the setpoint.
    // Start conservative; increase kP until responsive, then add kD to kill bounce.
    public static final double kP = 3.0;
    public static final double kI = 0.0;
    public static final double kD = 0.3;

    // ArmFeedforward gains (output = duty cycle).
    // kS: static friction — minimum duty cycle to break stiction.
    // kG: peak gravity — duty cycle to hold the arm level at its horizontal (worst-case) position.
    //     ArmFeedforward multiplies this by cos(θ), so it automatically scales to zero at the
    //     balance point and reverses sign on the other side — no manual sign logic needed.
    // kV: velocity feedforward — duty cycle per (rot/s); keeps profile tracking tight.
    // kA: acceleration feedforward — leave 0 for a lightweight arm.
    public static final double kS = 0.01;
    public static final double kG = 0.05; // Negated: test sign direction for gravity compensation
    public static final double kV = 0.01;
    public static final double kA = 0.01;

    // Gravity geometry — maps raw encoder rotations to a physical angle in radians
    // so that ArmFeedforward's cos(θ) term is geometrically correct.
    //
    // The arm passes through three characteristic encoder positions:
    //   kGravityPeakPosition  (0.079): arm is deployed (down) → maximum gravity torque, cos(θ) = 1
    //   kGravityZeroPosition  (0.23): arm is weightless (horizontal-ish) → zero gravity torque, cos(θ) = 0
    //   gravity peak again    (0.381): arm is retracted (up) → gravity reverses, cos(θ) = −1
    //
    // The conversion formula used in IntakeSubsystemProfiled:
    //   angleRad = (encoderPos − kGravityZeroPosition)
    //            / (kGravityZeroPosition − kGravityPeakPosition)
    //            * (π / 2)
    //
    // This produces:  0 rad at the peak (gravity max), π/2 rad at vertical (gravity zero), π rad on the far side —
    // exactly what ArmFeedforward expects for its cos(θ) calculation.
    public static final double kGravityPeakPosition = 0.079; // rotations (arm deployed, gravity max)
    public static final double kGravityZeroPosition = 0.23; // rotations (arm weightless, gravity zero)

    // isAtTarget() tolerance (rotations).
    public static final double kTolerance = 0.017; // ~6 degrees
  }

  public static final class LauncherConstants {
    // CAN IDs
    public static final int kFeederMotorCanId  = 50; // NEO 550 on SparkMax
    public static final int kShooterLeaderCanId   = 51; // Vortex on SparkFlex (leader)
    public static final int kShooterFollowerCanId = 52; // Vortex on SparkFlex (follower, inverted)

    // Motor inversion
    public static final boolean kFeederMotorInverted   = false;
    public static final boolean kShooterLeaderInverted = false;
    // Follower is physically inverted (same shaft, opposite facing motor)

    // Current limits (Amps)
    public static final int kFeederCurrentLimit  = 20;  // NEO 550
    public static final int kShooterCurrentLimit = 60;  // Vortex (each)

    // Feeder voltage (open-loop, full bus voltage for max RPM)
    public static final double kFeederVoltage = 12.0; // volts

    // Shooter control loop runs entirely on the roboRIO (WPILib PIDController +
    // SimpleMotorFeedforward) and sends kDutyCycle to the SparkFlex, bypassing
    // the REV firmware velocity loop which incorrectly idles at setpoint.

    // SimpleMotorFeedforward gains (voltage-based, RPM units)
    // kS: static friction (V) — minimum voltage to overcome stiction
    // kV: volts per RPM of steady-state velocity  (12V / 6784 RPM free speed)
    // kA: volts per RPM/s of acceleration — leave 0, flywheel inertia handles it
    public static final double kS_Shooter = 0.0;
    public static final double kV_Shooter = 12.3 / 6784.0; // V / RPM 12 seems a bit low 4500 target reacheds 4400 12.5 5580
    public static final double kA_Shooter = 0.0;

    // WPILib PIDController gains (output = Volts, input = RPM error)
    // kP: volts per RPM of error — keep small, kV handles steady state
    public static final double kP_Shooter = 0.001; // .0006 no oscillations
    public static final double kI_Shooter = 0.0;
    public static final double kD_Shooter = 0.0;

    // Target shooter speeds (RPM) — two presets for short and long distance shots
    public static final double kShooterTargetRpm = 2900.0;  // Default (for backwards compatibility)
    public static final double kShooterShortRpm = 2900.0;   // Close-range shot
    public static final double kShooterLongRpm  = 3450.0;   // Long-range shot

    // Tolerance for "at speed" check (RPM)
    public static final double kShooterRpmTolerance = 150.0;

    // MAXMotion velocity control gains (used by LauncherSubsystemMaxMotion)
    // kP_MM: duty-cycle per RPM of error — keep very small; kFF handles steady-state
    // kFF_MM: feedforward fraction = 1 / (free speed RPM).  Vortex free speed ≈ 6784 RPM.
    //         This drives ~1.0 duty cycle at free speed and scales linearly with setpoint.
    public static final double kP_MM_Shooter  = 0.0001;
    public static final double kI_MM_Shooter  = 0.0;
    public static final double kD_MM_Shooter  = 0.0;
    public static final double kFF_MM_Shooter = 1.0 / 6784.0; // duty-cycle / RPM

    // MAXMotion profile constraints for velocity mode
    // maxVelocity: maximum velocity the profile can reach (RPM). Set at/above your
    //              target so the profile never clamps the command.
    // maxAcceleration: RPM/s — how quickly the setpoint ramps up. Higher = faster spool,
    //                  but may cause current spikes. Start at ~3000 and tune up.
    public static final double kMaxVelocity_Shooter     = 6000.0;  // RPM (≥ max target RPM)
    public static final double kMaxAccel_Shooter        = 4000.0;  // RPM/s
    public static final double kAllowedError_Shooter    = 50.0;    // RPM — profile re-plan hysteresis
  }

  public static final class IndexerConstants {
    // CAN ID
    public static final int kIndexerMotorCanId = 40; // NEO 1.1 on SparkMax

    // Motor inversion
    public static final boolean kIndexerMotorInverted = true;

    // Current limit (Amps)
    public static final int kIndexerMotorCurrentLimit = 40; // NEO 1.1

    // Indexer voltage (open-loop, 12V for full speed)
    public static final double kIndexerVoltage = 12.0; // volts
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
