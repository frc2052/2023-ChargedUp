package frc.robot;

import com.team2052.lib.DrivetrainState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.io.Dashboard;

public class RobotState {
    private static RobotState INSTANCE;

    private Pose2d initialPose;
    private Pose2d robotPose;
    private Translation2d visionTranslation2d;
    private double detectionTime;
    private Rotation2d navxOffset;
    private Rotation2d robotRotation2d;
    private SwerveModulePosition[] swerveModulePositions;

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

     /*
     * RobotState keeps track of the position of the robot during a match
     */

    public void addDrivetrainState(DrivetrainState drivetrainState){
        this.swerveModulePositions = drivetrainState.getModulePositions();
        this.robotRotation2d = drivetrainState.getRotation2d();
    }

    public void addVisionTranslation3dUpdate(Translation3d robotVisionTranslation3d, double detectionTime){
        visionTranslation2d = robotVisionTranslation3d.toTranslation2d();
        this.detectionTime = detectionTime;
    }

    public void updateRobotPose(Pose2d robotPose){
        this.robotPose = robotPose;
    }

    public void reset(Pose2d initialStartingPose){
        navxOffset = new Rotation2d();
        navxOffset = initialStartingPose.getRotation();
        initialPose = initialStartingPose;
    }

    // Getters 

    public Translation2d getVisionTranslation2d(){
        return visionTranslation2d;
    }

    public double getVisionDetectionTime(){
        return detectionTime;
    }

    public Rotation2d getRotation2d(){
        return robotRotation2d.rotateBy(navxOffset);
    }

    public SwerveModulePosition[] getModulePositions(){
        return swerveModulePositions;
    }

    public Pose2d getRobotPose(){
        return robotPose;
    }

    public Pose2d getInitialPose(){
        return initialPose;
    }

    /**
     * Returns true if the robot is disabled.
     *
     * @return True if the robot is disabled.
     */
    public static boolean isDisabled() {
        return DriverStation.isDisabled();
    }

    /**
     * Returns true if the robot is enabled.
     *
     * @return True if the robot is enabled.
     */
    public static boolean isEnabled() {
        return DriverStation.isEnabled();
    }

    /**
     * Returns true if the robot is E-stopped.
     *
     * @return True if the robot is E-stopped.
     */
    public static boolean isEStopped() {
        return DriverStation.isEStopped();
    }

    /**
     * Returns true if the robot is in teleop mode.
     *
     * @return True if the robot is in teleop mode.
     */
    public static boolean isTeleop() {
        return DriverStation.isTeleop();
    }

    /**
     * Returns true if the robot is in autonomous mode.
     *
     * @return True if the robot is in autonomous mode.
     */
    public static boolean isAutonomous() {
        return DriverStation.isAutonomous();
    }

    /**
     * Returns true if the robot is in test mode.
     *
     * @return True if the robot is in test mode.
     */
    public static boolean isTest() {
        return DriverStation.isTest();
    }

    public void outputRobotStateToDashboard(){
        Dashboard.getInstance().putData("Rotation Degrees", robotRotation2d.getDegrees());
        //Dashboard.getInstance().putData("Position of Pose Estimator X", poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation(), getModulePositions()).getTranslation().getX());
        //Dashboard.getInstance().putData("Position of Pose Estimator Y", poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation(), getModulePositions()).getTranslation().getY());
        // Dashboard.getInstance().putData("Robot Pose X", robotPose2d.getX());
    }

    private RobotState() {}
     
}
