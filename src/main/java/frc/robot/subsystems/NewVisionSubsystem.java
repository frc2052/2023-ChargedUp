// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;

public class NewVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    
    /** Creates a new NewVisionSubsystem. */
    public NewVisionSubsystem() {
        camera = new PhotonCamera(Constants.Camera.CAMERA_NAME);
        
        SmartDashboard.putNumber("Pipeline", 0);
    }

    @Override
    public void periodic() {
        enableLEDs();

        camera.setPipelineIndex((int) SmartDashboard.getNumber("Pipeline", 0));
        //camera.setDriverMode(false);

        Dashboard.getInstance().putData("Camera Connected", camera.isConnected());
    }

    public void enableLEDs() {
        camera.setLED(VisionLEDMode.kOn);
    }

    public void disableLEDs() {
        camera.setLED(VisionLEDMode.kOff);
    }

    public PhotonTrackedTarget getAprilTagTarget() {
        camera.setPipelineIndex(0);

        return getTarget();
    }

    public PhotonTrackedTarget getReflectiveTarget() {
        camera.setPipelineIndex(1);

        return getTarget();
    }

    private PhotonTrackedTarget getTarget() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return result.getBestTarget();
        }

        return null;
    }

    public static double getDistanceToTargetMeters(PhotonTrackedTarget target) {
        try {
            return PhotonUtils.calculateDistanceToTargetMeters(
                Constants.Camera.CAMERA_POSITION_METERS.getZ(),
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile).getTagPose(
                    target.getFiducialId()
                ).get().getZ(),
                Constants.Camera.CAMERA_POSITION_METERS.getRotation().getY(),
                Units.degreesToRadians(target.getPitch())
            );
        } catch (IOException e) {
            return 0;
        }
    }

    public static Translation2d getRobotToTargetTranslationMeters(PhotonTrackedTarget target) {
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
