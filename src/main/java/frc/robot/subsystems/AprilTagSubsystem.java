package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    if(raspberryPiCameraPoseSubscriber.get().length == 3){
    
      Translation3d cameraTranslation3dMeters = new Translation3d(raspberryPiCameraPoseSubscriber.get()[0], raspberryPiCameraPoseSubscriber.get()[1], raspberryPiCameraPoseSubscriber.get()[2]);
      Rotation2d visionYaw = new Rotation2d(raspberryPiCameraPoseSubscriber.get()[3]);
      Pose2d robotVisionPose2d = new Pose2d((cameraTranslation3dMeters.plus(Constants.PiCamera.PI_CAMERA_POSITION_METERS.getTranslation())).toTranslation2d(), visionYaw);

      RobotState.getInstance().addVisionPose2dUpdate(robotVisionPose2d);

    }
  }
}
