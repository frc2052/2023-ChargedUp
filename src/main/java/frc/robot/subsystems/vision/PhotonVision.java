// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("2052April");

  Thread m_visionThread;
  public PhotonVision() {

    camera.takeInputSnapshot();// Capture pre-process camera stream image

    camera.takeOutputSnapshot();// Capture post-process camera stream image

      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
          // First calculate range
          double range =
                  PhotonUtils.calculateDistanceToTargetMeters(
                          0,
                          0,
                          0,
                          Units.degreesToRadians(result.getBestTarget().getPitch()));
      }
    }

    @Override
    public void periodic() {
      System.out.println(camera.isConnected());
    }
}
