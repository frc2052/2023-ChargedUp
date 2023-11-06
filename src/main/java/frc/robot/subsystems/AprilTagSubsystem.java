package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
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
    double[] x = new double[0];
    raspberryPiCameraPoseSubscriber = Dashboard.getInstance().getRaspberryPiTableTopic("cameraPoseMeters").subscribe(x);
  }

  public void update() {
    // This method will be called once per scheduler run
    if(raspberryPiCameraPoseSubscriber.get().length == 3){
      Translation3d cameraTranslation3dMeters = new Translation3d(raspberryPiCameraPoseSubscriber.get()[0], raspberryPiCameraPoseSubscriber.get()[1], Constants.PiCamera.Z_OFFSET_INCHES);
      Translation3d robotVisionTranslation3d = cameraTranslation3dMeters.minus(Constants.PiCamera.PI_CAMERA_POSITION_METERS.getTranslation());
      double detectionTime = raspberryPiCameraPoseSubscriber.get()[2];

      RobotState.getInstance().addVisionTranslation3dUpdate(robotVisionTranslation3d, detectionTime);
    }
  }
}
