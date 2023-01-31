// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
public class DynamicAutoFactory {
    private final DrivetrainSubsystem drivetrain;

    private Pose2d lastEndingPose;

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

        boolean onBlueAlliance = DriverStation.getAlliance() == Alliance.Blue;

        double invert = onBlueAlliance ? Constants.Auto.COMMUNITY_WIDTH_METERS - 5.48 : 0.0;
        

        Pose2d initialStartingPose = new Pose2d(
            getXAdjusted(getBaseLineXMeters(startingGrid, startingNode), onBlueAlliance), 
            Constants.Auto.ROBOT_LENGTH_METERS / 2,
            Rotation2d.fromDegrees(180)
            
        );
        
        
        Translation2d interpolationMidPoint = null;
        Pose2d launchPose = null;

        double launchYMeters = Constants.Auto.COMMUNITY_DEPTH_METERS + Constants.Auto.ROBOT_LENGTH_METERS;

        switch (exitChannel) {
            case LEFT_CHANNEL:
                double leftChannelXMeters = getXAdjusted(Constants.Auto.COMMUNITY_WIDTH_METERS, onBlueAlliance) - (Constants.Auto.CHANNEL_WIDTH_METERS / 2);

                interpolationMidPoint = new Translation2d(
                    leftChannelXMeters,
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2
                );

                launchPose = new Pose2d(
                    leftChannelXMeters,
                    launchYMeters,
                    new Rotation2d()
                );
                break;
        
            case RIGHT_CHANNEL:
            double rightChannelXMeters = Constants.Auto.CHANNEL_WIDTH_METERS / 2;

                interpolationMidPoint = new Translation2d(
                    getXAdjusted(rightChannelXMeters, onBlueAlliance),
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2
                );

                launchPose = new Pose2d(
                    getXAdjusted(rightChannelXMeters, onBlueAlliance),
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
                (gamePiece.ordinal() * Constants.Auto.DISTANCE_BETWEEN_GAME_PIECES_METERS) + 
                    Constants.Auto.DISTANCE_WALL_TO_GAME_PIECE_METERS,
                Constants.Auto.DISTANCE_GRID_TO_GAME_PIECES_METERS,
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
                new ParallelDeadlineGroup(gamePieceSwerveCommand, null)
            );

            if (scoreGamePiece) {
                Pose2d scorePose = new Pose2d(
                    getBaseLineXMeters(scoreGrid, scoreNode), 
                    Constants.Auto.ROBOT_LENGTH_METERS / 2,
                    Rotation2d.fromDegrees(0)
                );

                SwerveControllerCommand scoreSwerveCommand = createSwerveCommand(
                    lastEndingPose,
                    List.of(
                        new Translation2d(launchPose.getX(), launchPose.getY()),
                        interpolationMidPoint
                    ),
                    scorePose, 
                    new Rotation2d()
                );
                autoCommand.addCommands(scoreSwerveCommand);
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

            Translation2d lastTranslation = new Translation2d(lastEndingPose.getX(), lastEndingPose.getY());
            if (lastTranslation.getDistance(farEntryPoint) > lastTranslation.getDistance(nearEntryPoint)) {
                entryPoint = nearEntryPoint;
            } else {
                entryPoint = farEntryPoint;
            }

            SwerveControllerCommand endChargeStationSwerveCommand = createSwerveCommand(
                lastEndingPose,
                List.of(entryPoint),
                new Pose2d(
                    Constants.Auto.COMMUNITY_WIDTH_METERS / 2,
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS + (Constants.Auto.CHARGE_STATION_DEPTH/2),
                    new Rotation2d()
                ),
                new Rotation2d()
            );
            autoCommand.addCommands(endChargeStationSwerveCommand);
        } 

        return autoCommand;
    }

    private double getXAdjusted(double originalX, boolean onBlueAlliance){
        if (!onBlueAlliance) {
            return originalX;  
        } else {
            return Constants.Auto.FIELD_WIDTH - originalX;
        }
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
