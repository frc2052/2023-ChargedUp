// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.io.Dashboard.Channel;
import frc.robot.io.Dashboard.GamePiece;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class DynamicAutoFactory {
    private final DrivetrainSubsystem drivetrain;

    private Pose2d lastEndingPose;

    private Pose2d initialStartingPose;

    public DynamicAutoFactory(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    public SequentialCommandGroup getAuto(
        Grid startingGrid, 
        Node startingNode, 
        Channel exitChannel, 
        GamePiece gamePiece,
        boolean scoreGamePiece,
        Grid scoreGrid,
        Node scoreNode,
        Channel enterChannel,
        boolean endChargeStation
    ) {
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        Pose2d initialStartingPose = new Pose2d(
            getBaseLineYMeters(startingGrid, startingNode), 
            Constants.Auto.ROBOT_LENGTH_METERS / 2,
            Rotation2d.fromDegrees(180)
        );
        
        Translation2d interpolationMidPoint = null;
        Pose2d launchPose = null;

        double launchYMeters = Constants.Auto.COMMUNITY_DEPTH_METERS + Constants.Auto.ROBOT_LENGTH_METERS;

        switch (exitChannel) {
            case LEFT_CHANNEL:
                double leftChannelYMeters = Constants.Auto.COMMUNITY_WIDTH_METERS - (Constants.Auto.CHANNEL_WIDTH_METERS / 2);

                interpolationMidPoint = new Translation2d(
                    leftChannelYMeters,
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2
                );

                launchPose = new Pose2d(
                    leftChannelYMeters,
                    launchYMeters,
                    new Rotation2d()
                );
                break;
        
            case RIGHT_CHANNEL:
            double rightChannelYMeters = Constants.Auto.CHANNEL_WIDTH_METERS / 2;

                interpolationMidPoint = new Translation2d(
                    rightChannelYMeters,
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2
                );

                launchPose = new Pose2d(
                    rightChannelYMeters,
                    launchYMeters,
                    new Rotation2d()
                );
                break;
        }
        
        SwerveControllerCommand initialSwerveCommand = createSwerveCommand(
            initialStartingPose,
            List.of(interpolationMidPoint),
            launchPose,
            Rotation2d.fromDegrees(180)
        );

        autoCommand.addCommands(initialSwerveCommand);

        if (gamePiece != GamePiece.NO_GAME_PIECE) {
            Pose2d gamePieceEndPose = new Pose2d(
                Constants.Auto.DISTANCE_GRID_TO_GAME_PIECES_METERS,
                (gamePiece.ordinal() * Constants.Auto.DISTANCE_BETWEEN_GAME_PIECES_METERS) + 
                    Constants.Auto.DISTANCE_WALL_TO_GAME_PIECE_METERS,
                new Rotation2d()
            );

            SwerveControllerCommand gamePieceSwerveCommand = createSwerveCommand(
                lastEndingPose, 
                new ArrayList<Translation2d>(),
                gamePieceEndPose, 
                new Rotation2d()
            );

            autoCommand.addCommands(
                // TODO: Replace null with intake command
                // new ParallelDeadlineGroup(gamePieceSwerveCommand, null)
                gamePieceSwerveCommand
            );

            if (scoreGamePiece) {
                Pose2d scorePose = new Pose2d(
                        getBaseLineYMeters(scoreGrid, scoreNode),
                        Constants.Auto.ROBOT_LENGTH_METERS / 2,
                        Rotation2d.fromDegrees(0)

                );

                SwerveControllerCommand scoreSwerveCommand = createSwerveCommand(
                        lastEndingPose,
                        List.of(
                                new Translation2d(launchPose.getY(), launchPose.getX()),
                                interpolationMidPoint),
                        scorePose,
                        new Rotation2d());
                autoCommand.addCommands(scoreSwerveCommand);
            }

        }
        

        if (endChargeStation) {
            Translation2d farEntryPoint = new Translation2d(
                Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS + Constants.Auto.CHARGE_STATION_DEPTH,
                Constants.Auto.COMMUNITY_WIDTH_METERS / 2
            );

            Translation2d nearEntryPoint = new Translation2d(
                Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS,
                Constants.Auto.COMMUNITY_WIDTH_METERS / 2
            );

            Translation2d entryPoint = null;

            Translation2d lastTranslation = new Translation2d(lastEndingPose.getY(), lastEndingPose.getY());
            if (lastTranslation.getDistance(farEntryPoint) > lastTranslation.getDistance(nearEntryPoint)) {
                entryPoint = nearEntryPoint;
            } else {
                entryPoint = farEntryPoint;
            }

            SwerveControllerCommand endChargeStationSwerveCommand = createSwerveCommand(
                lastEndingPose,
                List.of(entryPoint),
                new Pose2d(
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS + (Constants.Auto.CHARGE_STATION_DEPTH/2),
                    Constants.Auto.COMMUNITY_WIDTH_METERS / 2,
                    new Rotation2d()
                ),
                new Rotation2d()
            );
            autoCommand.addCommands(endChargeStationSwerveCommand);
        } 

        return autoCommand;
    }

    private double getBaseLineYMeters(Grid grid, Node node) {
        return Units.inchesToMeters((3.5 + 16.5) + (3 * grid.ordinal() + node.ordinal()) * (18.5 + 13.5));
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
            new TrapezoidProfile.Constraints(
                drivetrain.getMaxAngularVelocityRadiansPerSecond(), 
                Math.PI
            )
        );

        lastEndingPose = endPose;

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
}
