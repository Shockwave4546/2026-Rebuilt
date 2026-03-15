// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Degrees;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_pigeon = new Pigeon2(DriveConstants.kGyroCanId, CANBus.roboRIO());

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  // Speed multiplier for dashboard control
  private double m_speedMultiplier = 1.0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Configure and zero the Pigeon2 gyro
    m_pigeon.setYaw(0);

    // Initialize odometry after gyro is ready
    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getGyroAngleSafe()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getGyroAngleSafe()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Put heading on dashboard
    SmartDashboard.putNumber("Heading", getHeading());

    // Read speed multiplier from dashboard
    m_speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", 1.0);
    m_speedMultiplier = Math.max(0.0, Math.min(1.0, m_speedMultiplier)); // Clamp between 0 and 1
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroAngleSafe()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Apply speed multiplier to translation only — rotation is intentionally excluded
    // so heading controllers (CoordinatedHeadingCommand, HoldHeadingCommand) retain
    // full authority regardless of the driver-selected speed limit.
    xSpeed *= m_speedMultiplier;
    ySpeed *= m_speedMultiplier;
    
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getGyroAngleSafe()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_pigeon.setYaw(0);
  }

  /**
   * Safely gets the gyro angle in degrees with null-safety check.
   * Returns 0.0 if the gyro angle is null, throws an exception, or is NaN/Infinite.
   *
   * @return the gyro angle in degrees, or 0.0 if unavailable
   */
  private double getGyroAngleSafe() {
    try {
      var yaw = m_pigeon.getYaw();
      if (yaw == null) {
        return 0.0;
      }
      double degrees = yaw.getValue().in(Degrees);
      return Double.isFinite(degrees) ? degrees : 0.0;
    } catch (Exception e) {
      return 0.0;
    }
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    try {
      var yaw = m_pigeon.getYaw();
      if (yaw == null) {
        return 0.0;
      }
      double degrees = yaw.getValue().in(Degrees);
      return Double.isFinite(degrees) ? degrees : 0.0;
    } catch (Exception e) {
      return 0.0;
    }
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    try {
      var angVelZ = m_pigeon.getAngularVelocityZWorld();
      if (angVelZ == null) {
        return 0.0;
      }
      double degreesPerSecond = angVelZ.getValue().in(edu.wpi.first.units.Units.DegreesPerSecond);
      return Double.isFinite(degreesPerSecond) ? degreesPerSecond * (DriveConstants.kGyroReversed ? -1.0 : 1.0) : 0.0;
    } catch (Exception e) {
      return 0.0;
    }
  }

  /**
   * Returns the Z-axis angular velocity from the gyro, in degrees per second.
   *
   * @return Z angular velocity in degrees per second
   */
  public double getAngularVelocityZ() {
    try {
      var angVelZ = m_pigeon.getAngularVelocityZWorld();
      if (angVelZ == null) {
        return 0.0;
      }
      double degreesPerSecond = angVelZ.getValue().in(edu.wpi.first.units.Units.DegreesPerSecond);
      return Double.isFinite(degreesPerSecond) ? degreesPerSecond * (DriveConstants.kGyroReversed ? -1.0 : 1.0) : 0.0;
    } catch (Exception e) {
      return 0.0;
    }
  }
  /**
   * Sets the speed multiplier.
   *
   * @param multiplier the speed multiplier (0.0 to 1.0)
   */
  public void setSpeedMultiplier(double multiplier) {
    m_speedMultiplier = Math.max(0.0, Math.min(1.0, multiplier));
  }

  /**
   * Applies thrust expo (exponential response curve) to stick input.
   * Provides fine control at low stick inputs while maintaining full speed capability.
   *
   * @param input the raw stick input (-1.0 to 1.0)
   * @return the expo-modified input
   */
  public static double applyThrustExpo(double input) {
    double sign = Math.signum(input);
    double absInput = Math.abs(input);
    double expo = DriveConstants.kThrustExpo;
    
    // Apply expo curve: preserves sign, applies cubic response with linear blend
    return sign * (Math.pow(absInput, 3.0) * (expo - 1.0) / expo + absInput / expo);
  }

}
