package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTrajectoryConfig {
    public static final AutoTrajectoryConfig slowTrajectoryConfig = new AutoTrajectoryConfig(
        new TrajectoryConfig(2.5 / 2, 1.5 / 2).setKinematics(DrivetrainSubsystem.getKinematics()),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(
            2, 0, 0,
            new TrapezoidProfile.Constraints(
                DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond(), 
                Math.PI
            )
        )
    );
    public static final AutoTrajectoryConfig fastTurnTrajectoryConfig = new AutoTrajectoryConfig(
        // Speed of actions, 1st TrajectoryFactory value is max velocity and 2nd is max accelaration.
        new TrajectoryConfig(3, 1.5).setKinematics(DrivetrainSubsystem.getKinematics()),
        // The XY controller PID value
        new PIDController(1, 0, 0),
        // Turning PID COntroller. Increasing 1st value increases speed of turning, and the TrapezoidalProfile is our contraints of these values.
        new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI))
    );
    public static final AutoTrajectoryConfig fastTurnSlowDriveTrajectoryConfig = new AutoTrajectoryConfig(
        new TrajectoryConfig(2, 1.5).setKinematics(DrivetrainSubsystem.getKinematics()), 
        new PIDController(0.25, 0, 0),
        new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4 * Math.PI, 3 * Math.PI))
    );
    public static final AutoTrajectoryConfig speedDriveTrajectoryConfig = new AutoTrajectoryConfig(
        new TrajectoryConfig(4.5, 3.5).setKinematics(DrivetrainSubsystem.getKinematics()), 
        new PIDController(1, 0, 0),
        new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4 * Math.PI, 3 * Math.PI))
    );
    
    private final TrajectoryConfig trajectoryConfig;
    private final PIDController XYController;
    private final ProfiledPIDController thetaController;

    /**
     * Creates a custom Trajectory Config from AutoTrajectoryConfig
     * @param maxXYVelocityMPS - Max driving velocity in meters per second
     * @param maxAccelarationMPS - Max driving accelaration in meters per second
     * @param xyP - Driving P (Proportional) value in PID. Increasing makes driving actions happen faster and decreasing makes them slower.
     * @param turnP - Driving P value. Increasing makes turning actions happen faster and decreasing makes them slower.
     * @param turnProfileContraintsMultiplier - Amount PI will be multiplied by in the TrapezoidalProfile.Constrains of the turning PID controller
     * @param startVelocityMPS - Tarjectory's starting velocity in meters per second
     * @param endVelocityMPS - Trajectory's ending velocity in meters per second
     * @return AutoTrajectoryConfig
     */
    public AutoTrajectoryConfig(
        double maxXYVelocityMPS, 
        double maxAccelarationMPS, 
        double xyP, 
        double turnP,
        double turnProfileContraintsMultiplier,
        double startVelocityMPS,
        double endVelocityMPS
    ) {
        this(
            new TrajectoryConfig(maxXYVelocityMPS, maxAccelarationMPS).setEndVelocity(endVelocityMPS).setStartVelocity(startVelocityMPS),
            new PIDController(xyP, 0, 0),
            new ProfiledPIDController(turnP, 0, 0, new TrapezoidProfile.Constraints(turnProfileContraintsMultiplier*Math.PI, turnProfileContraintsMultiplier*Math.PI))
        );
    }

    /**
     * Creates a custom Trajectory Config from AutoTrajectoryConfig without a specific start and end velocity.
     * @param maxXYVelocityMPS - Max driving velocity in meters per second
     * @param maxAccelarationMPS - Max driving accelaration in meters per second
     * @param xyP - Driving P (Proportional) value in PID. Increasing makes driving actions happen faster and decreasing makes them slower.
     * @param turnP - Driving P value. Increasing makes turning actions happen faster and decreasing makes them slower.
     * @param turnProfileContraintsMultiplier - Amount PI will be multiplied by in the TrapezoidalProfile.Constrains of the turning PID controller
     * @return AutoTrajectoryConfig
     */
    public AutoTrajectoryConfig(
        double maxXYVelocityMPS, 
        double maxAccelarationMPS, 
        double xyP, 
        double turnP, 
        double turnProfileContraintsMultiplier
    ) {
        this(
            new TrajectoryConfig(maxXYVelocityMPS, maxAccelarationMPS),
            new PIDController(xyP, 0, 0),
            new ProfiledPIDController(turnP, 0, 0, new TrapezoidProfile.Constraints(turnProfileContraintsMultiplier*Math.PI, turnProfileContraintsMultiplier*Math.PI))
        );
    }

    public AutoTrajectoryConfig(
        TrajectoryConfig trajectoryConfig, 
        PIDController XYController, 
        ProfiledPIDController thetController
    ) {
        this.trajectoryConfig = trajectoryConfig;
        this.XYController = XYController;
        this.thetaController = thetController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public AutoTrajectoryConfig withStartVelocity(double startVelocityMPS) {
        return new AutoTrajectoryConfig(trajectoryConfig.setStartVelocity(startVelocityMPS), XYController, thetaController);
    }

    public AutoTrajectoryConfig withEndVelocity(double endVelocityMPS) {
        return new AutoTrajectoryConfig(trajectoryConfig.setEndVelocity(endVelocityMPS), XYController, thetaController);
    }

    public AutoTrajectoryConfig withStartAndEndVelocity(double startVelocityMPS, double endVelocityMPS) {
        return new AutoTrajectoryConfig(trajectoryConfig.setStartVelocity(startVelocityMPS).setEndVelocity(endVelocityMPS), XYController, thetaController);
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return trajectoryConfig;
    }

    public PIDController getXYController() {
        return XYController;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }
}
