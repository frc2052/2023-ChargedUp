// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    
    private PhotonTrackedTarget lastResult;
    private Timer lastTargetTime;

    /** Creates a new NewVisionSubsystem. */
    public VisionSubsystem() {
        camera = new PhotonCamera(Constants.Camera.CAMERA_NAME);
        
        lastTargetTime = new Timer();
        lastTargetTime.start();

        disableLEDs();
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(Constants.Dashboard.CAMERA_CONNECTION_KEY, camera.isConnected());
    }

    public void enableLEDs() {
        camera.setLED(VisionLEDMode.kOn);
    }

    public void disableLEDs() {
        camera.setLED(VisionLEDMode.kOff);
    }

    public PhotonTrackedTarget getAprilTagTarget() {
        camera.setPipelineIndex(Constants.Camera.APRIL_TAG_PIPELINE);

        return getTarget();
    }

    public PhotonTrackedTarget getReflectiveTarget() {
        camera.setPipelineIndex(Constants.Camera.REFLECTIVE_TAPE_PIPELINE);

        return getTarget();
    }

    private PhotonTrackedTarget getTarget() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget bestResult = result.getBestTarget();

            if (bestResult.getPitch() < 5) {
                lastResult = bestResult;
                lastTargetTime.reset();
            }
        }

        if (lastTargetTime.get() <= 0.25) {
            return lastResult;
        }

        return null;
    }

    public static double getDistanceToTargetMeters(PhotonTrackedTarget target) {
        if (target == null) {
            return 0;
        }

        try {
            Optional<Pose3d> targetPose = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2023ChargedUp.m_resourceFile
            ).getTagPose(target.getFiducialId());

            double targetHeightMeters = 0;

            if (!targetPose.isEmpty()) {
                targetHeightMeters = targetPose.get().getZ();
            } else {
                targetHeightMeters = Units.inchesToMeters(24);
            }

            return PhotonUtils.calculateDistanceToTargetMeters(
                Constants.Camera.CAMERA_POSITION_METERS.getZ(),
                targetHeightMeters,
                Constants.Camera.CAMERA_POSITION_METERS.getRotation().getY(),
                Units.degreesToRadians(target.getPitch())
            );
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }

        return 0;
    }

    public static Translation2d getRobotToTargetTranslationMeters(PhotonTrackedTarget target) {
        if (target == null) {
            return new Translation2d();
        }

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
}
