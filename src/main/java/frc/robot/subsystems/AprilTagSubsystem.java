package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.io.Dashboard;

public class AprilTagSubsystem{
  static AprilTagSubsystem INSTANCE;

  private DoubleArraySubscriber raspberryPiCameraPoseSubscriber;
  
  public static AprilTagSubsystem getInstance(){
    if (INSTANCE == null){
      INSTANCE = new AprilTagSubsystem();
    } 
    
    return INSTANCE;
  }

  private AprilTagSubsystem() {
    double[] cameraPoseMeters = new double[0];
    raspberryPiCameraPoseSubscriber = Dashboard.getInstance().getRaspberryPiCameraPoseMeters().subscribe(cameraPoseMeters);
  }

  public void update() {
      Translation3d cameraTranslation2dMeters = new Translation3d(raspberryPiCameraPoseSubscriber.get()[0], raspberryPiCameraPoseSubscriber.get()[1], raspberryPiCameraPoseSubscriber.get()[2]);
      double visionYaw = raspberryPiCameraPoseSubscriber.get()[3];
      Pose2d robotVisionPose2d = new Pose2d(
        cameraTranslation2dMeters.plus(Constants.PiCamera.PI_CAMERA_POSITION_METERS.getTranslation()).toTranslation2d(), 
        new Rotation2d(Units.degreesToRadians(visionYaw))
      );

      System.out.println(visionYaw);

      RobotState.getInstance().addVisionPose2dUpdate(robotVisionPose2d);
  }
}
