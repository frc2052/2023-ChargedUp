// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public abstract class AutoBase extends SequentialCommandGroup {
    protected final DrivetrainSubsystem drivetrain;
    protected final ElevatorSubsystem elevator;
    protected final IntakeSubsystem intake;
    
    protected final AutoTrajectoryConfig slowTrajectoryConfig;
    protected final AutoTrajectoryConfig fastTurnTrajectoryConfig;
    protected final AutoTrajectoryConfig fastTurnSlowDriveTrajectoryConfig;
    protected final AutoTrajectoryConfig speedDriveTrajectoryConfig;

    private Pose2d lastEndingPose;
    
    /** Creates a new Auto. */
    public AutoBase(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, IntakeSubsystem intake) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        addRequirements(this.drivetrain);
        
        init();



        // PreConfigured trajectory configs

        slowTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2.5, 1.5).setKinematics(this.drivetrain.getKinematics()), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI))
        );

        fastTurnTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(3, 1.5).setKinematics(this.drivetrain.getKinematics()), // Speed of actions, 1st TrajectoryFactory value is max velocity and 2nd is max accelaration.
            new PIDController(1, 0, 0),  // The XY controller PID value
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 4*Math.PI)) // Turning PID COntroller. Increasing 1st value increases speed of turning, and the TrapezoidalProfile is our contraints of these values.
        );

        fastTurnSlowDriveTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2, 1.5).setKinematics(this.drivetrain.getKinematics()), 
            new PIDController(0.25, 0, 0),
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 3*Math.PI))
        );

        speedDriveTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(4.5, 3.5).setKinematics(this.drivetrain.getKinematics()), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 3*Math.PI))
        );

    }

    /**
     * Add commands to auto sequence from this method.
     */
    protected abstract void init();

    public Pose2d getLastEndingPose() {
        return lastEndingPose;
    }

    protected SwerveControllerCommand createSwerveCommand(
        Pose2d startPose,
        Pose2d endPose,
        Rotation2d rotation
    ) {
        return createSwerveCommand(
            startPose, 
            new ArrayList<Translation2d>(), 
            endPose, 
            rotation
        );
    }

    protected SwerveControllerCommand createSwerveCommand(
        Pose2d startPose, 
        List<Translation2d> midpoints,
        Pose2d endPose,
        Rotation2d rotation
    ) {
        return createSwerveTrajectoryCommand(slowTrajectoryConfig, startPose, midpoints, endPose, () -> rotation);
    }
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
    protected AutoTrajectoryConfig createTrajectoryConfig(double maxXYVelocityMPS, double maxAccelarationMPS, double xyP, double turnP, double turnProfileContraintsMultiplier, double startVelocityMPS, double endVelocityMPS) {
        return new AutoTrajectoryConfig(
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
    protected AutoTrajectoryConfig createTrajectoryConfig(double maxXYVelocityMPS, double maxAccelarationMPS, double xyP, double turnP, double turnProfileContraintsMultiplier) {
        return new AutoTrajectoryConfig(
            new TrajectoryConfig(maxXYVelocityMPS, maxAccelarationMPS),
            new PIDController(xyP, 0, 0),
            new ProfiledPIDController(turnP, 0, 0, new TrapezoidProfile.Constraints(turnProfileContraintsMultiplier*Math.PI, turnProfileContraintsMultiplier*Math.PI))
            );
    }

    // Swerve controller command generator method. Gives desired arguments to the SwerveControllerCommand
        // class that ultimatley tells the swerve drive robot where to drive and how to do so.

        // IMPORTANT THING TO KNOW ABOUT THE SWERVER CONTROLLER COMMAND: This command does not decide to
        // end by measuring the robot's actual position and measure if it hit the right point or not,
        // rather it calculates the expected amount of time that the path it's taking should take (highly accurate)
        // and ends after that specific amount of time has passed.

        protected SwerveControllerCommand createSwerveTrajectoryCommand(
            AutoTrajectoryConfig trajectoryConfig, 
            Pose2d startPose, 
            Pose2d endPose, 
            List<Translation2d> midpointList,
            Supplier<Rotation2d> rotationSupplier
        ) {
            lastEndingPose = endPose;
    
            return new SwerveControllerCommand(
                TrajectoryGenerator.generateTrajectory(
                    startPose,
                    midpointList,
                    endPose,
                    trajectoryConfig.getTrajectoryConfig()
                ),
                drivetrain::getPosition,
                this.drivetrain.getKinematics(),
                trajectoryConfig.getXYController(),
                trajectoryConfig.getXYController(),
                trajectoryConfig.getThetaController(), 
                rotationSupplier,
                drivetrain::setModuleStates,
                drivetrain
            );
        }

    // -- Different method versions containing different arguments)

    // Most basic deafult swerve command, automatically using slowTrajectoryConfig.
    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        Pose2d startPose, 
        Pose2d endPose
    ) {
        return createSwerveTrajectoryCommand(
            slowTrajectoryConfig,
            startPose,
            endPose,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose
    ) {
        return createSwerveTrajectoryCommand(
            trajectoryConfig,
            startPose,
            endPose,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose,
        List<Translation2d> midpointList, 
        Pose2d endPose
    ) {
        return createSwerveTrajectoryCommand(
            trajectoryConfig,
            startPose,
            endPose,
            midpointList,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {

        return createSwerveTrajectoryCommand(trajectoryConfig, startPose, new ArrayList<Translation2d>(), endPose, rotationSupplier);
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        List<Translation2d> midpointList,
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {
        lastEndingPose = endPose;

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                midpointList,
                endPose,
                trajectoryConfig.getTrajectoryConfig()
            ),
            drivetrain::getPosition,
            this.drivetrain.getKinematics(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(), 
            rotationSupplier,
            drivetrain::setModuleStates,
            drivetrain
        );
    }

    // Creates easy rotation 2d suppliers for the SwerveControllerCommands
    protected Supplier<Rotation2d> createRotationAngle(double angleDegrees) {
        return () -> { return Rotation2d.fromDegrees(angleDegrees); };
    }

}
