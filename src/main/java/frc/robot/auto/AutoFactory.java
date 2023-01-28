// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.function.Supplier;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutoFactory {
    private final DrivetrainSubsystem drivetrain;

    public AutoFactory(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    public SequentialCommandGroup getAuto(Grid grid, Node node, Channel channel) {
        double startingXMeters = Units.inchesToMeters((3.5 + 16.5) + (3 * grid.ordinal() + node.ordinal()) * (18.5 + 13.5));
        double startingYMeters = (Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS + (2 * Constants.Auto.BUPPER_DEPTH_METERS)) / 2;
        
        Pose2d initialStartingPose = new Pose2d(
            startingXMeters, 
            startingYMeters,
            Rotation2d.fromDegrees(0)
        );
        
        Translation2d initialMidPoint = null;
        Pose2d initialEndPose = null;

        switch (channel) {
            case LEFT_CHANNEL:
                double leftChannelXMeters = Constants.Auto.COMMUNITY_WIDTH_METERS - (Constants.Auto.CHANNEL_WIDTH / 2);

                initialMidPoint = new Translation2d(
                    leftChannelXMeters,
                    Constants.Auto.DISTANCE_CHARGE_STATION_GRID_METETERS / 2
                );

                initialEndPose = new Pose2d(
                    leftChannelXMeters,
                    0,
                    Rotation2d.fromDegrees(0)
                );
                break;
        
            case RIGHT_CHANNEL:
                double rightChannelXMeters = Constants.Auto.CHANNEL_WIDTH / 2;

                initialMidPoint = new Translation2d(
                    rightChannelXMeters,
                    Constants.Auto.DISTANCE_CHARGE_STATION_GRID_METETERS / 2
                );

                initialEndPose = new Pose2d(
                    rightChannelXMeters,
                    0,
                    Rotation2d.fromDegrees(0)
                );
                break;
        }

        SwerveControllerCommand initialSwerveCommand = createSwerveCommand(
            initialStartingPose,
            List.of(initialMidPoint),
            initialEndPose,
            new Rotation2d(180)
        );

        return null;
    }

    private SwerveControllerCommand createSwerveCommand(
        Pose2d startPose, 
        List<Translation2d> midpoints,
        Pose2d endPose,
        Rotation2d rotation
    ) {
        TrajectoryConfig config = new TrajectoryConfig(
            2.5,
            1.5
        ).setKinematics(drivetrain.getKinematics());

        PIDController xyController = new PIDController(1, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
            10,
            0,
            0,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                midpoints,
                endPose,
                config
            ),
            drivetrain::getPosition,
            drivetrain.getKinematics(),
            xyController,
            xyController,
            thetaController,
            () -> rotation,
            drivetrain::setModuleStates,
            drivetrain
        );
    }

    public static enum Grid {
        LEFT_GRID,
        CO_OP,
        RIGHT_GRID;
    }

    public static enum Node {
        LEFT_CONE, 
        MIDDLE_CONE, 
        RIGHT_CUBE;
    }

    public static enum Channel {
        LEFT_CHANNEL, 
        RIGHT_CHANNEL;
    }
}
