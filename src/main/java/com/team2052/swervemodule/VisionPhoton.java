package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPhoton extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("2052April");

  Thread m_visionThread;
  public VisionPhoton() {
    
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
