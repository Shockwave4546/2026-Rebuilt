// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.util.TelemetryRateLimiter;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Vision subsystem for tracking targets using PhotonVision cameras.
 * Manages two cameras: FrontLeft and FrontRight.
 */
public class VisionSubsystem extends SubsystemBase {
  // Camera instances
  private final PhotonCamera m_frontLeftCamera;
  private final PhotonCamera m_frontRightCamera;

  // Camera names (must match PhotonVision network table names)
  private static final String FRONT_LEFT_CAMERA_NAME = "FrontLeft";
  private static final String FRONT_RIGHT_CAMERA_NAME = "FrontRight";

  // Target tracking
  private PhotonTrackedTarget m_lastFrontLeftTarget = null;
  private PhotonTrackedTarget m_lastFrontRightTarget = null;

  /** Rate limiter for telemetry updates (10Hz instead of 50Hz). */
  private final TelemetryRateLimiter m_telemetryRateLimiter = new TelemetryRateLimiter(10.0);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    m_frontLeftCamera = new PhotonCamera(FRONT_LEFT_CAMERA_NAME);
    m_frontRightCamera = new PhotonCamera(FRONT_RIGHT_CAMERA_NAME);

    // Disable version check on cameras to reduce unnecessary network traffic
    PhotonCamera.setVersionCheckEnabled(false);
    
    System.out.println("VisionSubsystem initialized with cameras:");
    System.out.println("  - " + FRONT_LEFT_CAMERA_NAME + " (AprilTag detection only)");
    System.out.println("  - " + FRONT_RIGHT_CAMERA_NAME + " (AprilTag detection only)");
    System.out.println("  - Driver Camera streaming enabled (FMS bandwidth optimized)");
  }

  @Override
  public void periodic() {
    // Update front left camera
    var frontLeftResult = m_frontLeftCamera.getLatestResult();
    if (frontLeftResult.hasTargets()) {
      m_lastFrontLeftTarget = frontLeftResult.getBestTarget();
    } else {
      m_lastFrontLeftTarget = null;
    }

    // Update front right camera
    var frontRightResult = m_frontRightCamera.getLatestResult();
    if (frontRightResult.hasTargets()) {
      m_lastFrontRightTarget = frontRightResult.getBestTarget();
    } else {
      m_lastFrontRightTarget = null;
    }

    // --- Dashboard telemetry (rate-limited to 10Hz, change-detection on continuous values) ---
    if (m_lastFrontLeftTarget != null) {
      int tagID = m_lastFrontLeftTarget.getFiducialId();
      double yaw = m_lastFrontLeftTarget.getYaw();
      double skew = m_lastFrontLeftTarget.getSkew();

      if (m_telemetryRateLimiter.hasChangedNumber("Vision/FrontLeft_TagID", tagID)) {
        SmartDashboard.putNumber("Vision/FrontLeft_TagID", tagID);
      }
      if (m_telemetryRateLimiter.hasChangedNumber("Vision/FrontLeft_Yaw", yaw)) {
        SmartDashboard.putNumber("Vision/FrontLeft_Yaw", yaw);
      }
      if (m_telemetryRateLimiter.hasChangedNumber("Vision/FrontLeft_Skew", skew)) {
        SmartDashboard.putNumber("Vision/FrontLeft_Skew", skew);
      }
    }

    if (m_lastFrontRightTarget != null) {
      int tagID = m_lastFrontRightTarget.getFiducialId();
      double yaw = m_lastFrontRightTarget.getYaw();
      double skew = m_lastFrontRightTarget.getSkew();

      if (m_telemetryRateLimiter.hasChangedNumber("Vision/FrontRight_TagID", tagID)) {
        SmartDashboard.putNumber("Vision/FrontRight_TagID", tagID);
      }
      if (m_telemetryRateLimiter.hasChangedNumber("Vision/FrontRight_Yaw", yaw)) {
        SmartDashboard.putNumber("Vision/FrontRight_Yaw", yaw);
      }
      if (m_telemetryRateLimiter.hasChangedNumber("Vision/FrontRight_Skew", skew)) {
        SmartDashboard.putNumber("Vision/FrontRight_Skew", skew);
      }
    }
  }

  /**
   * Gets the best target from the front left camera.
   *
   * @return Optional containing the best target, or empty if no targets
   */
  public Optional<PhotonTrackedTarget> getFrontLeftTarget() {
    return Optional.ofNullable(m_lastFrontLeftTarget);
  }

  /**
   * Gets the best target from the front right camera.
   *
   * @return Optional containing the best target, or empty if no targets
   */
  public Optional<PhotonTrackedTarget> getFrontRightTarget() {
    return Optional.ofNullable(m_lastFrontRightTarget);
  }

  /**
   * Checks if front left camera has a valid target.
   *
   * @return true if a target is detected, false otherwise
   */
  public boolean hasFrontLeftTarget() {
    return m_lastFrontLeftTarget != null;
  }

  /**
   * Checks if front right camera has a valid target.
   *
   * @return true if a target is detected, false otherwise
   */
  public boolean hasFrontRightTarget() {
    return m_lastFrontRightTarget != null;
  }

  /**
   * Gets all targets from front left camera.
   *
   * @return List of all targets from front left camera
   */
  public List<PhotonTrackedTarget> getFrontLeftAllTargets() {
    var result = m_frontLeftCamera.getLatestResult();
    return result.hasTargets() ? new ArrayList<>(result.getTargets()) : new ArrayList<>();
  }

  /**
   * Gets all targets from front right camera.
   *
   * @return List of all targets from front right camera
   */
  public List<PhotonTrackedTarget> getFrontRightAllTargets() {
    var result = m_frontRightCamera.getLatestResult();
    return result.hasTargets() ? new ArrayList<>(result.getTargets()) : new ArrayList<>();
  }

  /**
   * Gets yaw angle from front left camera target.
   *
   * @return yaw in degrees, or 0 if no target
   */
  public double getFrontLeftYaw() {
    return m_lastFrontLeftTarget != null ? m_lastFrontLeftTarget.getYaw() : 0;
  }

  /**
   * Gets pitch angle from front left camera target.
   *
   * @return pitch in degrees, or 0 if no target
   */
  public double getFrontLeftPitch() {
    return m_lastFrontLeftTarget != null ? m_lastFrontLeftTarget.getPitch() : 0;
  }

  /**
   * Gets yaw angle from front right camera target.
   *
   * @return yaw in degrees, or 0 if no target
   */
  public double getFrontRightYaw() {
    return m_lastFrontRightTarget != null ? m_lastFrontRightTarget.getYaw() : 0;
  }

  /**
   * Gets pitch angle from front right camera target.
   *
   * @return pitch in degrees, or 0 if no target
   */
  public double getFrontRightPitch() {
    return m_lastFrontRightTarget != null ? m_lastFrontRightTarget.getPitch() : 0;
  }

  /**
   * Gets area of the target from front left camera (0-100).
   *
   * @return target area as percentage, or 0 if no target
   */
  public double getFrontLeftTargetArea() {
    return m_lastFrontLeftTarget != null ? m_lastFrontLeftTarget.getArea() : 0;
  }

  /**
   * Gets area of the target from front right camera (0-100).
   *
   * @return target area as percentage, or 0 if no target
   */
  public double getFrontRightTargetArea() {
    return m_lastFrontRightTarget != null ? m_lastFrontRightTarget.getArea() : 0;
  }

  /**
   * Checks if front left camera is connected.
   *
   * @return true if camera is connected, false otherwise
   */
  public boolean isFrontLeftConnected() {
    return m_frontLeftCamera.isConnected();
  }

  /**
   * Checks if front right camera is connected.
   *
   * @return true if camera is connected, false otherwise
   */
  public boolean isFrontRightConnected() {
    return m_frontRightCamera.isConnected();
  }

  /**
   * Gets the PhotonCamera object for direct access to front left camera.
   * Use this if you need lower-level camera access.
   *
   * @return the front left PhotonCamera
   */
  public PhotonCamera getFrontLeftCamera() {
    return m_frontLeftCamera;
  }

  /**
   * Gets the PhotonCamera object for direct access to front right camera.
   * Use this if you need lower-level camera access.
   *
   * @return the front right PhotonCamera
   */
  public PhotonCamera getFrontRightCamera() {
    return m_frontRightCamera;
  }

  /**
   * Gets a specific target by fiducial ID from the front left camera.
   * Searches through all targets to find the one with matching ID.
   *
   * @param targetId the fiducial ID to search for
   * @return Optional containing the target with matching ID, or empty if not found
   */
  public Optional<PhotonTrackedTarget> getFrontLeftTargetById(int targetId) {
    var result = m_frontLeftCamera.getLatestResult();
    if (result.hasTargets()) {
      for (var target : result.getTargets()) {
        if (target.getFiducialId() == targetId) {
          return Optional.of(target);
        }
      }
    }
    return Optional.empty();
  }

  /**
   * Gets a specific target by fiducial ID from the front right camera.
   * Searches through all targets to find the one with matching ID.
   *
   * @param targetId the fiducial ID to search for
   * @return Optional containing the target with matching ID, or empty if not found
   */
  public Optional<PhotonTrackedTarget> getFrontRightTargetById(int targetId) {
    var result = m_frontRightCamera.getLatestResult();
    if (result.hasTargets()) {
      for (var target : result.getTargets()) {
        if (target.getFiducialId() == targetId) {
          return Optional.of(target);
        }
      }
    }
    return Optional.empty();
  }
}
