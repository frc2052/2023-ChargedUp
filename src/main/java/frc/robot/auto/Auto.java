// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public abstract class Auto extends SequentialCommandGroup {
    protected final DrivetrainSubsystem drivetrain;
    
    private Pose2d lastEndingPose;
    
    /** Creates a new Auto. */
    public Auto(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);
        
        init();
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
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            2.5,
            1.5
        ).setKinematics(drivetrain.getKinematics());

        PIDController translationController = new PIDController(1, 0, 0);

        ProfiledPIDController rotationController = new ProfiledPIDController(
            10,
            0,
            0,
            new TrapezoidProfile.Constraints(
                drivetrain.getMaxAngularVelocityRadiansPerSecond(), 
                Math.PI
            )
        );

        // Set the last ending pose so future commands can use it as a initial pose.
        lastEndingPose = endPose;

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                midpoints,
                endPose,
                trajectoryConfig
            ),
            drivetrain::getPosition,
            drivetrain.getKinematics(),
            translationController,
            translationController,
            rotationController,
            () -> rotation,
            drivetrain::setModuleStates,
            drivetrain
        );
    }
}
