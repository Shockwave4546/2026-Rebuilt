package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.0125, 0, 0.165)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(3.1, 0, 0.875)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class IntakePivotProfiled {
        public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();

        static {
            pivotConfig
                    .inverted(IntakeConstants.kIntakePivotMotorInverted)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(IntakeConstants.kIntakePivotCurrentLimit);
            pivotConfig.absoluteEncoder
                    .inverted(IntakeConstants.kIntakePivotEncoderInverted)
                    .positionConversionFactor(1.0) // rotations
                    .velocityConversionFactor(1.0 / 60.0) // rotations per second
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        }
    }

    public static final class LauncherFeeder {
        public static final SparkMaxConfig feederConfig = new SparkMaxConfig();

        static {
            feederConfig
                    .inverted(LauncherConstants.kFeederMotorInverted)
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(LauncherConstants.kFeederCurrentLimit);
        }
    }

    public static final class LauncherShooter {
        public static final SparkFlexConfig leaderConfig = new SparkFlexConfig();
        public static final SparkFlexConfig followerConfig = new SparkFlexConfig();

        static {
            leaderConfig
                    .inverted(LauncherConstants.kShooterLeaderInverted)
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(LauncherConstants.kShooterCurrentLimit);
            leaderConfig.encoder
                    .positionConversionFactor(1.0) // rotations
                    .velocityConversionFactor(1.0 / 60.0); // rotations per second (will be converted to RPM elsewhere)

            followerConfig
                    .inverted(true) // Follower is physically inverted
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(LauncherConstants.kShooterCurrentLimit)
                    .follow(LauncherConstants.kShooterLeaderCanId, true);
        }
    }
}
