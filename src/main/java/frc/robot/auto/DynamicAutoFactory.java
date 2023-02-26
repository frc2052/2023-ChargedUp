// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.io.Dashboard.Channel;
import frc.robot.io.Dashboard.GamePiece;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Generates auto path: score gamepiece, drive to pick up gamepiece, drive to grid, 
 * score gamepiece, drive to chargestation, and balance.
 * 
 * NOTE: coordinates are community centric so that the far left corner of the alliance
 * community is (0, 0).
 */
public class DynamicAutoFactory {
    private final DrivetrainSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;

    public DynamicAutoFactory(DrivetrainSubsystem drivetrain,ElevatorSubsystem elevator, IntakeSubsystem intake, ArmSubsystem arm) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;
    }

    public SequentialCommandGroup getAuto(DynamicAutoConfiguration configuration) {
        // Inline implementation of the abstract auto class to create an instance of auto.
        return new AutoBase(drivetrain, elevator, intake, arm) {
            @Override
            public void init() {
                // Initial starting position of the robot across the front of the grids.
                Pose2d initialPose = createPose2dInches(
                    Constants.Auto.ROBOT_LENGTH_INCHES / 2, 
                    getBaseLineYInches(configuration.getStartingGrid(), configuration.getStartingNode()), 
                    0
                );
                // Interpolation point used for avoiding the near corner of the charge station.
                Translation2d nearChargeStationInterpolationPoint = createTranslation2dInches(
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_INCHES / 2, 
                    getMidChannelYInches(configuration.getExitChannel())
                );
                // Interpolation point used for avoiding the far corner of the charge station.
                Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_INCHES + Constants.Auto.CHARGE_STATION_DEPTH_INCHES, 
                    getMidChannelYInches(configuration.getExitChannel())
                );
                // Launching position for all options of the dynamic auto.
                Pose2d launchPose = createPose2dInches(
                    Constants.Auto.FAR_COMMUNITY_DEPTH_INCHES + Constants.Auto.ROBOT_LENGTH_INCHES, 
                    getMidChannelYInches(configuration.getExitChannel()), 
                    0
                );
                // Final ending pose for the center of the charge station.
                Pose2d chargeStationPose = createPose2dInches(
                    Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_INCHES + (Constants.Auto.CHARGE_STATION_DEPTH_INCHES / 2), 
                    Constants.Auto.COMMUNITY_WIDTH_INCHES / 2, 
                    0
                );

                drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));

                addCommands(new TopScoreCommand(elevator, arm));
                addCommands(new ScoreCommand(intake, arm, elevator).withTimeout(1));

                // Drive to launch point where other commands can be carried out.
                SwerveControllerCommand initialToLaunchCommand = createSwerveTrajectoryCommand(
                    AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(2), 
                    initialPose, 
                    List.of(nearChargeStationInterpolationPoint),
                    launchPose, 
                    createRotation(0)
                );
                addCommands(initialToLaunchCommand);

                // Check to pick up another game piece as the first option from launch point.
                if (configuration.getGamePiece() != GamePiece.NO_GAME_PIECE) {
                    Pose2d gamePiecePose = createPose2dInches(
                        Constants.Auto.DISTANCE_GRID_TO_GAME_PIECES_INCHES, 
                        getGamePieceY(configuration.getGamePiece()), 
                        0
                    );

                    SwerveControllerCommand launchToGamePieceCommand = createSwerveTrajectoryCommand(
                        AutoTrajectoryConfig.defaultTrajectoryConfig.withStartVelocity(2), 
                        getLastEndingPose(), 
                        gamePiecePose, 
                        createRotation(0)
                    );

                    ParallelDeadlineGroup pickUpGamePieceGroup = new ParallelDeadlineGroup(
                        launchToGamePieceCommand,
                        new ArmOutCommand(arm),
                        new IntakeInCommand(intake)
                    );
                    addCommands(pickUpGamePieceGroup);

                    if (configuration.scoreGamePiece()) {
                        Pose2d scorePose = createPose2dInches(
                            Constants.Auto.ROBOT_LENGTH_INCHES / 2, 
                            getBaseLineYInches(configuration.getScoreGrid(), configuration.getScoreNode()), 
                            0
                        );

                        SwerveControllerCommand gamePiecetoScoreCommand = createSwerveCommand(
                            getLastEndingPose(),
                            List.of(
                                farChargeStationInterpolationPoint,
                                nearChargeStationInterpolationPoint
                            ),
                            scorePose,
                            createRotation(180)
                        );
                        addCommands(gamePiecetoScoreCommand);

                        addCommands(new MidScoreCommand(elevator, arm));
                        addCommands(new ScoreCommand(intake, arm, elevator).withTimeout(1));

                        if (configuration.endChargeStation()) {
                            Pose2d chargeStationNearLineUpPose = createPose2dInches(
                                Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_INCHES - (Constants.Auto.ROBOT_LENGTH_INCHES / 2), 
                                Constants.Auto.COMMUNITY_WIDTH_INCHES / 2, 
                                0
                            );

                            SwerveControllerCommand lineUpCommand = createSwerveTrajectoryCommand(
                                AutoTrajectoryConfig.defaultTrajectoryConfig, 
                                getLastEndingPose(), 
                                chargeStationNearLineUpPose,
                                createRotation(0)    
                            );
                            addCommands(lineUpCommand);
                        }
                    } else if (configuration.endChargeStation()) {
                        Pose2d chargeStationFarLineUpPose = createPose2dInches(
                            Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_INCHES - (Constants.Auto.ROBOT_LENGTH_INCHES / 2), 
                            Constants.Auto.COMMUNITY_WIDTH_INCHES / 2, 
                            0
                        );
                        Translation2d chargeStationFarInterpolationPoint = createTranslation2dInches(
                            Constants.Auto.DISTANCE_GRID_TO_GAME_PIECES_INCHES * 0.75, 
                            getMidChannelYInches(configuration.getExitChannel())
                        );

                        SwerveControllerCommand lineUpCommand = createSwerveTrajectoryCommand(
                            AutoTrajectoryConfig.defaultTrajectoryConfig, 
                            getLastEndingPose(),
                            List.of(chargeStationFarInterpolationPoint),
                            chargeStationFarLineUpPose,
                            createRotation(0)    
                        );
                        addCommands(lineUpCommand);
                    }

                    // Catch all drive onto charge station an balance command.
                    if (configuration.endChargeStation()) {
                        SwerveControllerCommand endChargeStationCommand = createSwerveTrajectoryCommand(
                            AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
                            getLastEndingPose(), 
                            chargeStationPose,
                            createRotation(0)
                        );
                        addCommands(endChargeStationCommand);
                        
                        addCommands(new ChargeStationBalanceCommand(drivetrain));
                    }
                }
            }
        };
    }

    private double getBaseLineYInches(Grid grid, Node node) {
        double distanceToFirstPipeInches = 3.5 + 16.5;
        int scoringPositionIndex = (3 * grid.ordinal()) + node.ordinal();
        double scoringElementWidthInches = 18.5 + 13.5;

        return distanceToFirstPipeInches + (scoringPositionIndex * scoringElementWidthInches);
    }

    private double getMidChannelYInches(Channel channel) {
        switch (channel) {
            case LEFT_CHANNEL:
                return Constants.Auto.COMMUNITY_WIDTH_INCHES;
        
            case RIGHT_CHANNEL:
                return Constants.Auto.COMMUNITY_WIDTH_INCHES -
                    (Constants.Auto.CHANNEL_WIDTH_INCHES / 2);
        }
        return 0;
    }

    private double getGamePieceY(GamePiece gamePiece) {
        return (gamePiece.ordinal() * Constants.Auto.DISTANCE_BETWEEN_GAME_PIECES_INCHES) + 
            Constants.Auto.DISTANCE_WALL_TO_GAME_PIECE_INCHES;
    }
}
