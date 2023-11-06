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
    private Translation3d visionTranslation3d;
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

    public void addDrivetrainState(DrivetrainState drivetrainState){
        this.swerveModulePositions = drivetrainState.getModulePositions();
        this.robotRotation2d = drivetrainState.getRotation2d();
    }

    /**
     * Adds an AprilTag vision tracked translation3d WITHOUT timestamp.
     */ 
    public void addVisionTranslation3dUpdate(Translation3d robotVisionTranslation3d){
        visionTranslation3d = robotVisionTranslation3d;
    }

    /**
     * Adds an AprilTag vision tracked translation3d WITH timestamp.
     */ 
    public void addVisionTranslation3dUpdate(Translation3d robotVisionTranslation3d, double detectionTime){
        visionTranslation3d = robotVisionTranslation3d;
        this.detectionTime = detectionTime;
    }

    public void updateRobotPose(Pose2d robotPose){
        this.robotPose = robotPose;
    }

    /**
     * Reset the RobotState's Initial Pose2d and set the NavX Offset. 
     * NavX offset is set when the robot has an inital rotation not facing where you want 0 (forwards) to be.
     */
    public void reset(Pose2d initialStartingPose){
        navxOffset = new Rotation2d();
        navxOffset = initialStartingPose.getRotation();
        initialPose = initialStartingPose;
    }

    /**
     * Returns the latest AprilTag vision detection robot translation in Translation2d
     * 
     * @return Translation2d
     */
    public Translation2d getVisionTranslation2d(){
        return visionTranslation3d.toTranslation2d();
    }

    /**
     * Returns the latest AprilTag Vision detection time. This is when (on the raspberry pi) the Translation3d was last updated.
     * 
     * @return double
     */
    public double getVisionDetectionTime(){
        return detectionTime;
    }

    /**
     * Returns the Rotation2d of the robot, accounting for an offset that was set when intialized. 
     * 
     * @return Rotation2d
     */
    public Rotation2d getRotation2d(){
        return robotRotation2d.rotateBy(navxOffset);
    }

    /**
     * Returns the SwerveModulePositions of the drivetrain swerve modules. 
     * 
     * @return SwerveModulePosition[]
     */
    public SwerveModulePosition[] getModulePositions(){
        return swerveModulePositions;
    }

    /**
     * Returns the Pose2d of the robot that was given by the estimator. 
     * 
     * @return Pose2d
     */
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
