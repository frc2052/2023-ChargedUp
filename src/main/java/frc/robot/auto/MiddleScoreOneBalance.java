// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Middle score gamepiece, drive to chargestation, and balance.
 */
public class MiddleScoreOneBalance extends AutoBase {
    public MiddleScoreOneBalance(
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
            0, 
            // Recenter offset to zero is the middle node
            getStartingYOffsetInches(
                autoConfiguration.getScoreGrid(), 
                autoConfiguration.getStartingNode()
            ) + (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES), 
            0
        );
        Pose2d lineUpPose = createPose2dInches(24, 0, 0);
        Pose2d chargeStationPose = createPose2dInches(100, 0, 0);
        Pose2d driveOverPose = createPose2dInches(172, 0, 0);
        Pose2d pickUpPose = createPose2dInches(234, -12, 0);
        Pose2d finalChargeStationPose = createPose2dInches(60, 0, 0);

        addCommands(new ResetOdometryCommand(drivetrain, initialPose));
        
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }
        
        addCommands(
            new ScoreCommand(intake, arm, elevator, autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE).withTimeout(
                autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.5
            )
        );
        
        if (autoConfiguration.endChargeStation()) {
            SwerveControllerCommand lineUpPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(1), 
                initialPose,
                lineUpPose,
                createRotation(180)
            );
            addCommands(lineUpPath);

            SwerveControllerCommand onChargePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig.withStartAndEndVelocity(1, 0), 
                getLastEndingPose(),
                chargeStationPose,
                createRotation(180)
            );
            addCommands(onChargePath);

            SwerveControllerCommand overChargePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.snailTrajectoryConfig, 
                getLastEndingPose(),
                driveOverPose,
                createRotation(180)
            );
            addCommands(overChargePath);

            SwerveControllerCommand pickUpPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig, 
                getLastEndingPose(),
                pickUpPose,
                createRotation(0)
            );
            ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
                pickUpPath,
                new ArmOutCommand(arm).andThen(new ElevatorPositionCommand(ElevatorPosition.GROUND_PICKUP, elevator)),
                new IntakeInCommand(intake)
            );
            addCommands(pickUpGroup);

            addCommands(new ArmInCommand(arm));

            SwerveControllerCommand chargeStationPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
                getLastEndingPose(),
                finalChargeStationPose,
                createRotation(0)
            );
            addCommands(chargeStationPath);

            addCommands(new ChargeStationBalanceCommand(drivetrain));
        }
    };
}
