// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonVision extends SubsystemBase {
  public PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  // private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0, 0.5), new Rotation3d(0,0,0));

  // private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);



  // boolean hasTargets = result.hasTargets();

  /** Creates a new PhotonVision. */
  public PhotonVision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean hasTarget() {
    PhotonPipelineResult result = camera.getLatestResult();
    
    return result.hasTargets();

    // List<PhotonTrackedTarget> target = result.getTargets();

    // return target.;
  }

  public int getBestTargetID() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (hasTarget()) {
      return result.getBestTarget().getFiducialId();      
    }

    return 0;
  }
  
  public PhotonTrackedTarget getBestTarget() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (hasTarget()) {
      return result.getBestTarget();      
    }

    return null;
  }

  public double getYaw() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      return target.getYaw();
    }

    return 0;
  }

  public double getDistance() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      return PhotonUtils.calculateDistanceToTargetMeters(0.40005, Units.inchesToMeters(36.5), Rotation2d.fromDegrees(25).getRadians(), Units.degreesToRadians(result.getBestTarget().getPitch()));
    }

    return 0;
  }
}
