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
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutoFactory {
    private final DrivetrainSubsystem drivetrain;

    private Pose2d lastPose;

    public AutoFactory(DrivetrainSubsystem drivetrain) {
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
        SequentialCommandGroup command = new SequentialCommandGroup();

        double robotLength = (Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS + (2 * Constants.Auto.BUPPER_DEPTH_METERS));
        
        Pose2d initialStartingPose = new Pose2d(
            getBaseLineXMeters(startingGrid, startingNode), 
            robotLength / 2,
            Rotation2d.fromDegrees(0)
        );
        
        Translation2d initialMidPoint = null;
        Pose2d initialEndPose = null;

        double endingYMeters = Constants.Auto.COMMUNITY_HEIGHT_METERS + (robotLength / 2);

        switch (exitChannel) {
            case LEFT_CHANNEL:
                double leftChannelXMeters = Constants.Auto.COMMUNITY_WIDTH_METERS - (Constants.Auto.CHANNEL_WIDTH_METERS / 2);

                initialMidPoint = new Translation2d(
                    leftChannelXMeters,
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2
                );

                initialEndPose = new Pose2d(
                    leftChannelXMeters,
                    endingYMeters,
                    Rotation2d.fromDegrees(0)
                );
                break;
        
            case RIGHT_CHANNEL:
                double rightChannelXMeters = Constants.Auto.CHANNEL_WIDTH_METERS / 2;

                initialMidPoint = new Translation2d(
                    rightChannelXMeters,
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2
                );

                initialEndPose = new Pose2d(
                    rightChannelXMeters,
                    endingYMeters,
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
        command.addCommands(initialSwerveCommand);

        if (gamePiece != GamePiece.NO_GAME_PIECE) {
            Pose2d gamePieceEndPose = new Pose2d(
                (gamePiece.ordinal() * Constants.Auto.DISTANCE_BETWEEN_GAME_PIECES_METERS) + Constants.Auto.DISTANCE_WALL_TO_GAME_PIECE_METERS,
                Constants.Auto.DISTANCE_GRID_TO_GAME_PIECES_METERS,
                new Rotation2d()
            );

            SwerveControllerCommand gamePieceSwerveCommand = createSwerveCommand(
                lastPose, 
                new ArrayList<Translation2d>(),
                gamePieceEndPose, 
                new Rotation2d()
            );
            // TODO: Make parallel command to run lower and run intake
            command.addCommands(gamePieceSwerveCommand);

            if (scoreGamePiece) {
                Pose2d scorePose = new Pose2d(
                    getBaseLineXMeters(scoreGrid, scoreNode), 
                    robotLength / 2,
                    Rotation2d.fromDegrees(0)
                );
            }
        }

        if (endChargeStation){
            Translation2d farEntryPoint = new Translation2d(
                Constants.Auto.COMMUNITY_WIDTH_METERS / 2,
                Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS + Constants.Auto.CHARGE_STATION_DEPTH
            );

            Translation2d nearEntryPoint = new Translation2d(
                Constants.Auto.COMMUNITY_WIDTH_METERS / 2,
                Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS
            );

            Translation2d entryPoint = null;

            Translation2d lastTranslation = new Translation2d(lastPose.getX(), lastPose.getY());
            if (lastTranslation.getDistance(farEntryPoint) > lastTranslation.getDistance(nearEntryPoint)) {
                entryPoint = nearEntryPoint;
            } else {
                entryPoint = farEntryPoint;
            }

            SwerveControllerCommand endChargeStationSwerveCommand = createSwerveCommand(
                lastPose,
                List.of(entryPoint),
                new Pose2d(
                    Constants.Auto.COMMUNITY_WIDTH_METERS / 2,
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS + (Constants.Auto.CHARGE_STATION_DEPTH/2),
                    new Rotation2d()
                ),
                new Rotation2d()
            );
            command.addCommands(endChargeStationSwerveCommand);
        } 

        return command;
    }

    private double getBaseLineXMeters(Grid grid, Node node) {
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
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );

        lastPose = endPose;

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

    public static enum GamePiece {
        FAR_LEFT_GAME_PIECE,
        MIDDLE_LEET_GAME_PIECE,
        MIDDLE_RIGHT_GAME_PIECE,
        FAR_RIGHT_GAME_PIECE,
        NO_GAME_PIECE;
    }
}
