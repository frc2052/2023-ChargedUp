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
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    private PhotonPipelineResult latestResult;
    
    public PhotonVisionSubsystem() {
        camera = new PhotonCamera(Constants.Camera.CAMERA_NAME);

        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.AVERAGE_BEST_TARGETS, 
                camera, 
                Constants.Camera.cameraPosition
            );
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }

        camera.setDriverMode(false);
        camera.setPipelineIndex(0);

        camera.setLED(VisionLEDMode.kOff);
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();

        SmartDashboard.putBoolean("Camera Connected", camera.isConnected());

        if (latestResult.hasTargets()) {
            SmartDashboard.putNumber("TAG ID", latestResult.getBestTarget().getFiducialId());
        }
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

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose) {
        if (poseEstimator == null) {
            return Optional.empty();
        }
        
        poseEstimator.setReferencePose(previousEstimatedRobotPose);
        return poseEstimator.update();
    }

    public static double getHorizontalOffsetDegrees(PhotonTrackedTarget target) {
        return target.getYaw();
    }

    /**
     * @return Estimated height from the ground to the center of the april tag in meters.
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
