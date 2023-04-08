// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;
import frc.robot.subsystems.IntakeSubsystem;

@AutoDescription(description = "Score gamepiece, drive over charge station to pick up second gamepiece, and balance.")
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
        final double startingYOffset = getStartingYOffsetInches(
            autoConfiguration.getScoreGrid(), 
            autoConfiguration.getStartingNode()
        ) + (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);

        // Recenter offset to zero is the middle node
        final Pose2d initialPose = createPose2dInches(0, startingYOffset, 0);
        final Pose2d lineUpPose = createPose2dInches(24, 0, 0);
        final Pose2d chargeStationPose = createPose2dInches(134, 0, 0);
        final Pose2d driveOverPose = createPose2dInches(200, 1, 0);
        final Pose2d finalChargeStationPose = createPose2dInches(120, 0, 180);

        final AutoTrajectoryConfig retractTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 1);
        final AutoTrajectoryConfig chargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 1, 0.5);
        final AutoTrajectoryConfig driveOverTrajectoryConfig = new AutoTrajectoryConfig(0.5, 1.5, 0.5, 3, 2, 0.5, 0);
        final AutoTrajectoryConfig rechargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 0, 0);

        addCommands(new ResetOdometryCommand(drivetrain, initialPose));
        
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }
        
        addCommands(new ScoreCommand(
            () -> ScoreMode.CONE, 
            () -> autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.5,
            intake
        ));
        
        SwerveControllerCommand lineUpPath = createSwerveTrajectoryCommand(
            retractTrajectoryConfig, 
            initialPose,
            lineUpPose,
            createRotation(180)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            lineUpPath.beforeStarting(new WaitCommand(0.5)),
            new CompleteScoreCommand(elevator, intake, arm)
        );
        addCommands(retractGroup);

        SwerveControllerCommand onChargePath = createSwerveTrajectoryCommand(
            chargeStationTrajectoryConfig, 
            getLastEndingPose(),
            chargeStationPose,
            createRotation(180)
        );
        ParallelDeadlineGroup onChargeGroup = new ParallelDeadlineGroup(
            onChargePath,
            new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator)
        );
        addCommands(onChargeGroup);

        SwerveControllerCommand overChargePath = createSwerveTrajectoryCommand(
            driveOverTrajectoryConfig, 
            getLastEndingPose(),
            driveOverPose,
            createRotation(0)
        );
        addCommands(overChargePath);

        if (autoConfiguration.endChargeStation()) {
            SwerveControllerCommand chargeStationPath = createSwerveTrajectoryCommand(
                rechargeStationTrajectoryConfig, 
                getLastEndingPose(),
                finalChargeStationPose,
                createRotation(0)
            );
            addCommands(chargeStationPath);

            addCommands(new ChargeStationBalanceCommand(drivetrain));
        }
    }
}
