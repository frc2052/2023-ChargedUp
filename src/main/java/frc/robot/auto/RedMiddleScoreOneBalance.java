// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Middle score gamepiece, drive to chargestation, and balance.
 */
public class RedMiddleScoreOneBalance extends AutoBase {
    public RedMiddleScoreOneBalance(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);
    }
    
    public void init() {
        Pose2d initialPose = createPose2dInches(
            Constants.Auto.ROBOT_LENGTH_INCHES / 2, 
            // Recenter offset to zero is the middle node
            getLeftStartingYOffsetInches(
                autoConfiguration.getStartingGrid(), 
                autoConfiguration.getStartingNode()
            ) - (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES), 
            0
        );
        Pose2d lineUpPose = createPose2dInches(
            Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_INCHES / 2, 
            0,
            0
        );
        Pose2d chargeStationPose = createPose2dInches(
            Constants.Auto.DISTANCE_GRID_TO_CHARGE_STATION_INCHES + (Constants.Auto.CHARGE_STATION_DEPTH_INCHES / 2), 
            0, 
            0
        );

        drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));

        addCommands(new MidScoreCommand(elevator, arm));
        addCommands(new ScoreCommand(intake, arm, elevator, true).withTimeout(1));
        
        if (autoConfiguration.endChargeStation()) {
            SwerveControllerCommand lineUpPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(1), 
                initialPose,
                lineUpPose,
                createRotation(180)
            );
            addCommands(lineUpPath);

            SwerveControllerCommand chargePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig.withStartVelocity(1), 
                getLastEndingPose(),
                chargeStationPose,
                createRotation(180)
            );
            addCommands(chargePath);

            addCommands(new ChargeStationBalanceCommand(drivetrain));
        }
    };
}
