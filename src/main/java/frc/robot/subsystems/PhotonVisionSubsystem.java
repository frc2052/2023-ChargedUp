// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;

    private PhotonPipelineResult latestResult;

    private final Transform3d robotToCamera;
    private PhotonPoseEstimator poseEstimator;

    public PhotonVisionSubsystem() {
        camera = new PhotonCamera(Constants.Camera.CAMERA_NAME);
        
        robotToCamera = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5), 
            new Rotation3d(0, 0, 0)
        );
        
        try {
            poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                camera, 
                robotToCamera
            );
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }

        camera.setDriverMode(true);
        camera.setPipelineIndex(0);
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    public boolean hasTargets() {
        return latestResult.hasTargets();
    }

    public PhotonTrackedTarget getTarget() throws TargetNotFoundException {
        if (!hasTargets()) {
            // If there aren't any targets stop any further vision processing.
            throw new TargetNotFoundException();
        }
        return latestResult.getBestTarget();
    }

    public static double getHorizontalOffsetMeters(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getY();
    }

    /**
     * @return Estimated height from the ground to the center of the april tag in
     *         meters.
     */
    public static double getTargetHeightFromGroundMeters(PhotonTrackedTarget target) {
        // Special case for april tags mounted in the loading zone,
        // which are higher than those in the community.
        if (target.getFiducialId() == 4 || target.getFiducialId() == 5) {
            return Constants.Camera.LOADING_ZONE_GROUND_TO_APRIL_TAG_HEIGHT_METERS +
                (Constants.Camera.APRIL_TAG_HEIGHT_METERS / 2);
        }

        return Constants.Camera.COMMUNITY_GROUND_TO_APRIL_TAG_HEIGHT_METERS +
            (Constants.Camera.APRIL_TAG_HEIGHT_METERS / 2);
    }
    
    /**
     * @return Estimated distance from the camera to the april tag in meters.
    */
    public static double getDistanceToTargetMeters(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            Constants.Camera.CAMERA_HEIGHT_METERS, 
            getTargetHeightFromGroundMeters(target), 
            Constants.Camera.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(target.getPitch())
        );
    }

    public class TargetNotFoundException extends Exception { }
}
