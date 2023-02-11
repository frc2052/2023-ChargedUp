// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("2052April");
  Transform3d robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);
 // Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), robotToCamera);
 private PhotonPipelineResult result;
  private PhotonTrackedTarget target;

  Thread m_visionThread;

  public PhotonVision() {
    camera.setDriverMode(true);
    camera.setPipelineIndex(0);

    try {
      aprilTagFieldLayout = AprilTagFieldLayout
      .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public int getID() {
    return this.getID();
  }

  public boolean hasTarget() {
    return result.hasTargets();
  } // Whether the pipeline is detecting targets or not.

  public double targetPitch() {
    return this.targetPitch();
  }// The pitch of the target in degrees (positive up).

  public double targetYaw() {
    return target.getYaw();
  } // The yaw of the target in degrees (positive right).

  public double targetArea() {
    return this.targetArea();
  } // The area (percent of bounding box in screen) as a percent (0-100).

  public double targetSkew() {
    return this.targetSkew();
  } // The skew of the target in degrees (counter-clockwise positive).

  public double targetPose() {
    return this.targetPose();
  } // The pose of the target relative to the robot (x, y, z, qw, qx, qy, qz)

  public double targetPixelsX() {
    return this.targetPixelsX();
  } // The target crosshair location horizontally, in pixels (origin top-right)

  public double targetPixelsY() {
    return this.targetPixelsY();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  

  @Override
  public void periodic() {
    camera.takeInputSnapshot();// Capture pre-process camera stream image

    camera.takeOutputSnapshot();// Capture post-process camera stream image

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult();

    List<PhotonTrackedTarget> targets = result.getTargets();

    target = result.getBestTarget();

    double pitch = target.getPitch();
    double area = target.getArea();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    System.out.println(camera.isConnected());
  }
}
