package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.io.Dashboard;

public class AprilTagSubsystem{
  static AprilTagSubsystem INSTANCE;

  private DoubleArraySubscriber raspberryPiCameraPoseSubscriber;

  private Translation3d lastCameraVisionUpdateTranslation3d;
  private Translation3d deadzoneTranslation3d = new Translation3d(0.05, 0.05, 0.05);
  
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
    if(raspberryPiCameraPoseSubscriber.get().length == 3){
    
      Translation3d cameraTranslation3dMeters = new Translation3d(raspberryPiCameraPoseSubscriber.get()[0], raspberryPiCameraPoseSubscriber.get()[1], raspberryPiCameraPoseSubscriber.get()[2]);

      if (lastCameraVisionUpdateTranslation3d == null){
        lastCameraVisionUpdateTranslation3d = cameraTranslation3dMeters;
      }

      if (cameraTranslation3dMeters != lastCameraVisionUpdateTranslation3d){
        Translation3d robotVisionTranslation3d = cameraTranslation3dMeters.minus(Constants.PiCamera.PI_CAMERA_POSITION_METERS);
        RobotState.getInstance().addVisionTranslation3dUpdate(robotVisionTranslation3d);
        lastCameraVisionUpdateTranslation3d = cameraTranslation3dMeters;
      } else {
        System.out.println("Stale Vision Reading");
      }
      


    //   if ((robotVisionTranslation3d.plus(deadzoneTranslation3d)).getZ() > lastrobotVisionUpdateTranslation3d.getZ() || (robotVisionTranslation3d.minus(deadzoneTranslation3d)).getZ() < lastrobotVisionUpdateTranslation3d.getZ()){
    //     RobotState.getInstance().addVisionTranslation3dUpdate(robotVisionTranslation3d);
    //     lastrobotVisionUpdateTranslation3d = robotVisionTranslation3d;
    //   } else {
    //     System.out.println("Stale Vision Reading");
    //   }
    }
  }
}
