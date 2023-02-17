package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class AutoTrajectoryConfig {
    private final TrajectoryConfig trajectoryConfig;
    private final PIDController XYController;
    private final ProfiledPIDController thetaController;

    public AutoTrajectoryConfig(TrajectoryConfig trajectoryConfig, PIDController XYController, ProfiledPIDController thetController) {
        this.trajectoryConfig = trajectoryConfig;
        this.XYController = XYController;
        this.thetaController = thetController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
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

    public AutoTrajectoryConfig withStartVelocity(double startVelocityMPS) {
        return new AutoTrajectoryConfig(trajectoryConfig.setStartVelocity(startVelocityMPS), XYController, thetaController);
    }

    public AutoTrajectoryConfig withEndVelocity(double endVelocityMPS) {
        return new AutoTrajectoryConfig(trajectoryConfig.setEndVelocity(endVelocityMPS), XYController, thetaController);
    }

    public AutoTrajectoryConfig withStartAndEndVelocity(double startVelocityMPS, double endVelocityMPS) {
        return new AutoTrajectoryConfig(trajectoryConfig.setStartVelocity(startVelocityMPS).setEndVelocity(endVelocityMPS), XYController, thetaController);
    }
}
