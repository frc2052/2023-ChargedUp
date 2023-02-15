// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.io.Dashboard.GamePiece;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class DynamicAutoFactory {
    private final DrivetrainSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;

    public DynamicAutoFactory(DrivetrainSubsystem drivetrain,ElevatorSubsystem elevator, IntakeSubsystem intake) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
    }

    public SequentialCommandGroup getAuto(DynamicAutoConfiguration configuration) {
        // Inline implementation of the abstract auto class to create an instance of auto.
        return new AutoBase(this.drivetrain,this.elevator,this.intake) {
            @Override
            protected void init() {
                // TODO: Start by automatically scoring the starting game piece.

                // Initial starting position of the robot across the front of the grids.
                Pose2d initialStartingPose = new Pose2d(
                    Constants.Auto.ROBOT_LENGTH_METERS / 2,
                    getBaseLineYMeters(configuration.getStartingGrid(), configuration.getStartingNode()), 
                    new Rotation2d()
                );

                drivetrain.resetOdometry(initialStartingPose);

                // Interpolation point used to avoid collisions with the charge station.
                Translation2d chargeStationInterpolationMidPoint = null;
                // Ending launch point to "launch" any of the next paths.
                Pose2d launchPointPose = null;
        
                double launchPointXMeters = Constants.Auto.COMMUNITY_DEPTH_METERS + 
                    Constants.Auto.ROBOT_LENGTH_METERS;

                // Switch between left and right launch points and interpolation points.
                switch (configuration.getExitChannel()) {
                    case LEFT_CHANNEL:
                        // Subtracting by the width of the community will flip our coordinates from right to left.
                        double leftChannelYMeters = Constants.Auto.COMMUNITY_WIDTH_METERS -
                            (Constants.Auto.CHANNEL_WIDTH_METERS / 2);
        
                        //a point that is half the distance between the charge station and grid (X)
                        //and half the distance between charge station and side wall (y)
                        chargeStationInterpolationMidPoint = new Translation2d(
                            Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2,
                            leftChannelYMeters
                        );

                        //point half way between wall and charge station (y) and just outside the community line (x)
                        launchPointPose = new Pose2d(
                            launchPointXMeters,
                            leftChannelYMeters,
                            new Rotation2d()
                        );
                        break;
                
                    case RIGHT_CHANNEL:
                        double rightChannelYMeters = Constants.Auto.CHANNEL_WIDTH_METERS / 2;
        
                        chargeStationInterpolationMidPoint = new Translation2d(
                            Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS / 2,
                            rightChannelYMeters
                        );

                        launchPointPose = new Pose2d(
                            launchPointXMeters,
                            rightChannelYMeters,
                            new Rotation2d()
                        );
                        break;
                }
                
                // Creates the initial swerve path to get to the launch path.
                SwerveControllerCommand initialSwerveCommand = createSwerveCommand(
                    initialStartingPose,
                    List.of(chargeStationInterpolationMidPoint),
                    launchPointPose,
                    // Sets 0 degrees as facing away from the grid and driver station.
                    Rotation2d.fromDegrees(180)
                );
                addCommands(initialSwerveCommand);
        
                // Check to pick up another game piece as the first option from launch point.
                if (configuration.getGamePiece() != GamePiece.NO_GAME_PIECE) {
                    Pose2d gamePieceEndPose = new Pose2d(
                        Constants.Auto.DISTANCE_GRID_TO_GAME_PIECES_METERS,
                        Constants.Auto.COMMUNITY_WIDTH_METERS - 
                            (configuration.getGamePiece().ordinal() * Constants.Auto.DISTANCE_BETWEEN_GAME_PIECES_METERS) + 
                            Constants.Auto.DISTANCE_WALL_TO_GAME_PIECE_METERS,
                        new Rotation2d()
                    );
        
                    SwerveControllerCommand gamePieceSwerveCommand = createSwerveCommand(
                        getLastEndingPose(),
                        gamePieceEndPose, 
                        new Rotation2d()
                    );
                    addCommands(gamePieceSwerveCommand);
        
                    // Check to score picked up game piece if pick up game piece was selected.
                    if (configuration.scoreGamePiece()) {
                        Pose2d scorePose = new Pose2d(
                            Constants.Auto.ROBOT_LENGTH_METERS / 2,
                            getBaseLineYMeters(configuration.getScoreGrid(), configuration.getScoreNode()),
                            new Rotation2d()
                        );

                        SwerveControllerCommand scoreSwerveCommand = createSwerveCommand(
                            getLastEndingPose(),
                            List.of(
                                new Translation2d(launchPointPose.getX(), launchPointPose.getY()),
                                chargeStationInterpolationMidPoint
                            ),
                            scorePose,
                            new Rotation2d()
                        );
                        addCommands(scoreSwerveCommand);
                    }
                }
                
                // Check to end by balancing on the charge station, otherwise end the auto command.
                if (configuration.endChargeStation()) {
                    // Entry point onto charge station nearest from the driver station.
                    Translation2d nearChargeStationEntryPoint = new Translation2d(
                        Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS,
                        Constants.Auto.COMMUNITY_WIDTH_METERS / 2
                    );

                    // Entry point onto charge station farthest from the driver station.
                    Translation2d farChargeStationEntryPoint = new Translation2d(
                        Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS + Constants.Auto.CHARGE_STATION_DEPTH,
                        Constants.Auto.COMMUNITY_WIDTH_METERS / 2
                    );
        
                    Translation2d chargeStationEntryPoint = null;
        
                    // Determine closest entry point onto the charge station.
                    Translation2d lastEndingTranslation = new Translation2d(getLastEndingPose().getX(), getLastEndingPose().getY());
                    if (lastEndingTranslation.getDistance(nearChargeStationEntryPoint) <= lastEndingTranslation.getDistance(farChargeStationEntryPoint)) {
                        chargeStationEntryPoint = nearChargeStationEntryPoint;
                    } else {
                        chargeStationEntryPoint = farChargeStationEntryPoint;
                    }
        
                    SwerveControllerCommand endChargeStationSwerveCommand = createSwerveCommand(
                        getLastEndingPose(),
                        List.of(chargeStationEntryPoint),
                        new Pose2d(
                            Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_METERS + (Constants.Auto.CHARGE_STATION_DEPTH / 2),
                            Constants.Auto.COMMUNITY_WIDTH_METERS / 2,
                            new Rotation2d()
                        ),
                        new Rotation2d()
                    );
                    addCommands(endChargeStationSwerveCommand);

                    // TODO: End with auto balance command.
                }
            }
        };
    }

    private double getBaseLineYMeters(Grid grid, Node node) {
        double distanceToFirstPipeInches = 3.5 + 16.5;
        int scoringPositionIndex = (3 * grid.ordinal()) + node.ordinal();
        double scoringElementWidthInches = 18.5 + 13.5;

        return Constants.Auto.COMMUNITY_WIDTH_METERS - 
            Units.inchesToMeters(distanceToFirstPipeInches + (scoringPositionIndex * scoringElementWidthInches));
    }
}
