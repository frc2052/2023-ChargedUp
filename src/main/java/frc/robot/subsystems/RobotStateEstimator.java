package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotState;

public class RobotStateEstimator{
    static RobotStateEstimator INSTANCE;

    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * RobotStateEstimator uses SwerveDrivePoseEstimator to estimate the pose of the robot, field relative.
     */
    public static RobotStateEstimator getInstance(){
        if (INSTANCE == null) {
            INSTANCE = new RobotStateEstimator();
        }

        return INSTANCE;
    }

    private RobotStateEstimator() {
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Drivetrain.kinematics, 
            RobotState.getInstance().getRotation2d(), 
            RobotState.getInstance().getModulePositions(),
            new Pose2d()
        );
    }

    /**
     * Update the SwerveDrivePoseEstimator with values from RobotState
     */
    public void updateRobotPoseEstimator() {
        poseEstimator.addVisionMeasurement(
            new Pose2d(
                RobotState.getInstance().getVisionTranslation2d(), 
                RobotState.getInstance().getRotation2d()
            ),
            RobotState.getInstance().getVisionDetectionTime()
        );
        
        poseEstimator.update(
            RobotState.getInstance().getRotation2d(), 
            RobotState.getInstance().getModulePositions()
        );

        RobotState.getInstance().updateRobotPose(poseEstimator.getEstimatedPosition());
        
    }

    /**
     * Reset the position of SwerveDrivePoseEstimator and set the NavX Offset
     */
    public void resetOdometry(Pose2d initialStartingPose){
        RobotState.getInstance().reset(initialStartingPose);

        poseEstimator.resetPosition(
            RobotState.getInstance().getRotation2d(), 
            RobotState.getInstance().getModulePositions(),
            initialStartingPose
        );
    }
}
