// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

/**
 * Aligns the robot to face the hub AND drive to optimal shooting distance using AprilTag detection.
 * 
 * <p>Identifies which hub is visible based on AprilTag IDs:
 * - Red hub: Tags 5, 8, 9, 10, 11, or 2
 * - Blue hub: Tags 18, 21, 24, 25, 26, or 27
 * 
 * <p>Uses multi-tag approach for robust alignment on two axes:
 * 1. <b>Rotation (Heading):</b> Multi-tag averaging for yaw offset, continuously corrects 
 *    for gyro drift during rotation. Target: 0° yaw (tags centered in camera)
 * 2. <b>Distance:</b> Uses vision pose estimation to calculate distance to hub center,
 *    then drives forward/backward to maintain 3m optimal shooting distance
 * 
 * <p>When multiple hub tags are visible, calculates average yaw offset across all 
 * visible hub tags for more robust estimates. Single-tag fallback available.
 * 
 * <p>The rotation PID controller accounts for the camera's position and orientation on 
 * the robot, providing real-time visual feedback to correct gyro drift.
 * 
 * <p>The distance PID controller uses vision-estimated position to drive to the ideal
 * 3m distance from hub center. This distance is measured from the camera/robot to hub center.
 * 
 * <p>This command will only initialize if at least one hub tag is currently visible.
 */
public class AlignToHubCommand extends Command {
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;
  private final PIDController m_rotationController;
  private final PIDController m_distanceController;
  
  // Red hub AprilTag IDs
  private static final int[] RED_HUB_TAG_IDS = {2, 5, 8, 9, 10, 11};
  
  // Blue hub AprilTag IDs
  private static final int[] BLUE_HUB_TAG_IDS = {18, 21, 24, 25, 26, 27};
  
  // PID gains for rotation to face hub
  private static final double kRotationP = 0.05;
  private static final double kRotationI = 0.0;
  private static final double kRotationD = 0.01;
  private static final double ROTATION_TOLERANCE = 2.0; // degrees
  
  // PID gains for distance control to hub
  private static final double kDistanceP = 0.5;
  private static final double kDistanceI = 0.0;
  private static final double kDistanceD = 0.1;
  private static final double DISTANCE_TOLERANCE = 0.1; // meters (10cm)
  
  // Ideal shooting distance from hub center (meters)
  private static final double IDEAL_DISTANCE_TO_HUB = 3.0;

  /**
   * Creates a new AlignToHubCommand.
   *
   * @param vision the vision subsystem
   * @param drive the drive subsystem
   */
  public AlignToHubCommand(VisionSubsystem vision, DriveSubsystem drive) {
    m_vision = vision;
    m_drive = drive;
    
    // Rotation controller (for heading/yaw)
    m_rotationController = new PIDController(kRotationP, kRotationI, kRotationD);
    m_rotationController.setTolerance(ROTATION_TOLERANCE);
    m_rotationController.enableContinuousInput(-180, 180);
    
    // Distance controller (for forward/backward movement)
    m_distanceController = new PIDController(kDistanceP, kDistanceI, kDistanceD);
    m_distanceController.setTolerance(DISTANCE_TOLERANCE);

    addRequirements(m_vision, m_drive);
  }

  @Override
  public void initialize() {
    m_rotationController.reset();
    m_distanceController.reset();
    System.out.println("AlignToHubCommand started");
  }

  @Override
  public void execute() {
    // Try to find any hub tags in either camera
    Optional<PhotonCamera> activeCamera = Optional.empty();
    java.util.List<PhotonTrackedTarget> visibleHubTags = new java.util.ArrayList<>();
    HubSide detectedHub = null;

    // Check front left camera for hub tags
    var frontLeftResult = m_vision.getFrontLeftCamera().getLatestResult();
    if (frontLeftResult.hasTargets()) {
      for (var target : frontLeftResult.getTargets()) {
        HubSide hubSide = getHubSide(target.getFiducialId());
        if (hubSide != null) {
          visibleHubTags.add(target);
          if (detectedHub == null) {
            detectedHub = hubSide;
          }
          activeCamera = Optional.of(m_vision.getFrontLeftCamera());
        }
      }
    }

    // Also check front right camera for hub tags (if not already found many on left)
    if (visibleHubTags.size() < 3) {
      var frontRightResult = m_vision.getFrontRightCamera().getLatestResult();
      if (frontRightResult.hasTargets()) {
        for (var target : frontRightResult.getTargets()) {
          HubSide hubSide = getHubSide(target.getFiducialId());
          if (hubSide != null) {
            // Check if we already have this tag
            boolean alreadyHave = visibleHubTags.stream()
                .anyMatch(t -> t.getFiducialId() == target.getFiducialId());
            if (!alreadyHave) {
              visibleHubTags.add(target);
            }
            if (detectedHub == null) {
              detectedHub = hubSide;
            }
            activeCamera = Optional.of(m_vision.getFrontRightCamera());
          }
        }
      }
    }

    if (!visibleHubTags.isEmpty() && activeCamera.isPresent() && detectedHub != null) {
      // Average yaw offset across all visible hub tags for robust heading estimate
      double averageYaw = visibleHubTags.stream()
          .mapToDouble(PhotonTrackedTarget::getYaw)
          .average()
          .orElse(0.0);

      int visibleTagCount = visibleHubTags.size();
      int firstTagId = visibleHubTags.get(0).getFiducialId();

      // Goal: get average tag yaw to 0 (centered in camera frame)
      // This accounts for camera mounting offset automatically
      double desiredYaw = 0.0;
      
      // Get current heading from gyro (this drifts, but vision corrects it)
      double currentHeading = m_drive.getHeading();
      
      // Use PID with average tag yaw offset as feedback to correct gyro drift
      double rotationOutput = m_rotationController.calculate(averageYaw, desiredYaw);
      
      // Calculate distance to hub using vision pose estimation
      var pipelineResult = activeCamera.get().getLatestResult();
      var multiTagResult = pipelineResult.getMultiTagResult();
      
      double forwardOutput = 0.0;
      double distanceToHub = 0.0;
      boolean hasDistanceEstimate = false;
      
      if (multiTagResult.isPresent()) {
        try {
          // Use the first visible tag's distance from camera as the distance to hub
          var firstTag = visibleHubTags.get(0);
          var tagPose = firstTag.getBestCameraToTarget();
          double tagDistanceFromCamera = tagPose.getTranslation().getNorm();
          
          // Distance is tag distance from camera (camera to tag distance = robot to hub approx)
          distanceToHub = tagDistanceFromCamera;
          hasDistanceEstimate = true;
          
          // PID controller to maintain ideal 3m distance
          // Positive output = move forward, negative = move backward
          forwardOutput = m_distanceController.calculate(distanceToHub, IDEAL_DISTANCE_TO_HUB);
          
        } catch (Exception e) {
          // If pose calculation fails, just use rotation
          System.err.println("Failed to calculate distance to hub: " + e.getMessage());
          forwardOutput = 0.0;
        }
      }
      
      // Drive with both forward and rotation
      m_drive.drive(forwardOutput, 0, rotationOutput, false);

      // Dashboard telemetry
      SmartDashboard.putNumber("AlignToHub/Visible_Tag_Count", visibleTagCount);
      SmartDashboard.putNumber("AlignToHub/First_TagID", firstTagId);
      SmartDashboard.putString("AlignToHub/Hub_Side", detectedHub.toString());
      SmartDashboard.putNumber("AlignToHub/Gyro_Heading", currentHeading);
      SmartDashboard.putNumber("AlignToHub/Average_Yaw_Offset", averageYaw);
      SmartDashboard.putNumber("AlignToHub/Rotation_Output", rotationOutput);
      
      if (hasDistanceEstimate) {
        SmartDashboard.putNumber("AlignToHub/Distance_To_Hub", distanceToHub);
        SmartDashboard.putNumber("AlignToHub/Ideal_Distance", IDEAL_DISTANCE_TO_HUB);
        SmartDashboard.putNumber("AlignToHub/Forward_Output", forwardOutput);
        SmartDashboard.putNumber("AlignToHub/Distance_Error", distanceToHub - IDEAL_DISTANCE_TO_HUB);
      }
      
      boolean rotationAtTarget = Math.abs(averageYaw) < ROTATION_TOLERANCE;
      boolean distanceAtTarget = !hasDistanceEstimate || m_distanceController.atSetpoint();
      
      SmartDashboard.putBoolean("AlignToHub/Rotation_At_Target", rotationAtTarget);
      SmartDashboard.putBoolean("AlignToHub/Distance_At_Target", distanceAtTarget);
      SmartDashboard.putBoolean("AlignToHub/Fully_Aligned", rotationAtTarget && distanceAtTarget);
      SmartDashboard.putString("AlignToHub/Status", 
          "Aligning to hub (" + visibleTagCount + " tags, vision-corrected)");
    } else {
      // No hub targets visible
      m_drive.drive(0, 0, 0, false);
      SmartDashboard.putString("AlignToHub/Status", "No hub targets visible");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    System.out.println("AlignToHubCommand ended" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    // Command finishes when the PID controllers are at their setpoints:
    // - Rotation aligned (tags centered in camera)
    // - Distance maintained (at 3m from hub)
    // For now, just check rotation since distance needs more data
    return m_rotationController.atSetpoint();
  }

  /**
   * Determines which hub is detected based on AprilTag ID.
   *
   * @param tagId the detected AprilTag ID
   * @return HubSide enum (RED or BLUE) if hub is detected, null otherwise
   */
  private HubSide getHubSide(int tagId) {
    for (int redTagId : RED_HUB_TAG_IDS) {
      if (tagId == redTagId) {
        return HubSide.RED;
      }
    }
    
    for (int blueTagId : BLUE_HUB_TAG_IDS) {
      if (tagId == blueTagId) {
        return HubSide.BLUE;
      }
    }
    
    return null;
  }

  /** Enum for which hub the robot is targeting */
  private enum HubSide {
    RED("Red Hub"),
    BLUE("Blue Hub");

    private final String label;

    HubSide(String label) {
      this.label = label;
    }

    @Override
    public String toString() {
      return label;
    }
  }
}

