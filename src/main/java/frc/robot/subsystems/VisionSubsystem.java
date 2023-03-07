// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;

    private final PhotonPoseEstimator poseEstimator;
    private static AprilTagFieldLayout fieldLayout;
    
    private final Timer driveModeResetTimer;

    public VisionSubsystem() {
        camera = new PhotonCamera(Constants.Camera.CAMERA_NAME);

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }

        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.AVERAGE_BEST_TARGETS, 
            camera,
            Constants.Camera.CAMERA_POSITION_METERS
        );

        camera.setLED(VisionLEDMode.kOff);

        driveModeResetTimer = new Timer();

        SmartDashboard.putNumber("Pipeline", 0);
    }

    @Override
    public void periodic() {
        // if (driveModeResetTimer.get() >= 1.0) {
        //     camera.setDriverMode(true);
        //     driveModeResetTimer.stop();
        // }

        camera.setPipelineIndex((int)SmartDashboard.getNumber("Pipeline", 0));
        //camera.setDriverMode(false);

        Dashboard.getInstance().putData("Camera Connected", camera.isConnected());
    }

    public void enableLED() {
        camera.setLED(VisionLEDMode.kOn);
    }

    public void disableLED() {
        camera.setLED(VisionLEDMode.kOff);
    }

    public PhotonTrackedTarget getReflectiveTarget() throws TargetNotFoundException {
        camera.setPipelineIndex(1);

        return getTarget();
    }
    
    public PhotonTrackedTarget getTarget() throws TargetNotFoundException {
        camera.setDriverMode(false);
        driveModeResetTimer.reset();

        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            // If there aren't any targets stop any further vision processing.
            throw new TargetNotFoundException();
        }

        return result.getBestTarget();
    }

    /**
     * @return Estimated distance from the camera to the april tag in meters.
    */
    public static double getDistanceToTargetMeters(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            Constants.Camera.CAMERA_POSITION_METERS.getZ(),
            fieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
            Constants.Camera.CAMERA_POSITION_METERS.getRotation().getY(),
            Units.degreesToRadians(target.getPitch())
        );
    }

    public static Translation2d getRobotToTargetTranslation(PhotonTrackedTarget target) throws IOException {
        // Offset of camera to target
        Translation2d cameraToTarget = PhotonUtils.estimateCameraToTargetTranslation(
            getDistanceToTargetMeters(target),
            Rotation2d.fromDegrees(-target.getYaw())
        );

        // Convert camera relative to robot relative
        return new Translation2d(
            cameraToTarget.getX() + Constants.Camera.CAMERA_POSITION_METERS.getX(), 
            cameraToTarget.getY() + Constants.Camera.CAMERA_POSITION_METERS.getY()
        );
    }

    public class TargetNotFoundException extends Exception { }
}
