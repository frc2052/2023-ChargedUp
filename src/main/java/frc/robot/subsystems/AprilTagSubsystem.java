package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team2052.lib.PiCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;
import frc.robot.RobotState;

public class AprilTagSubsystem{
    public static AprilTagSubsystem INSTANCE;

    private RobotState robotState = RobotState.getInstance();
    private List<PiCamera> cameras = new ArrayList<PiCamera> ();
    //private PiCamera[] cameras = new PiCamera[2];
    private Pose2d averagePose2dMeters;
  
    public static AprilTagSubsystem getInstance(){
        if (INSTANCE == null){
            INSTANCE = new AprilTagSubsystem();
        } 
        
        return INSTANCE;
    }

    private AprilTagSubsystem() {
        cameras.add(new PiCamera("PiCamera1", Constants.PiCamera1.PI_CAMERA_POSITION_METERS));
        cameras.add(new PiCamera("PiCamera2", Constants.PiCamera2.PI_CAMERA_POSITION_METERS));
    }

    public void update() {
        Pose2d total = new Pose2d();

        for(int i = 0; i < cameras.size(); i++){
            PiCamera camera = cameras.get(i);
            if(camera.isValid()){
                total.plus(new Transform2d(total, camera.getEstimatedPosition()));
            } else {
                System.out.println(camera.getName() + " IS INVALID");
            }
        }

        averagePose2dMeters = total.div(cameras.size());
        robotState.addVisionPose2dUpdate(averagePose2dMeters);
    }
}
