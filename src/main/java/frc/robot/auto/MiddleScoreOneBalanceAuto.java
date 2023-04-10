// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.GamePiece;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

@AutoDescription(description = "Score gamepiece, drive over charge station to pick up second gamepiece, and balance.")
@DashboardAutoRequirements(requirements = { Node.class, GamePiece.class, ChargeStation.class })
public class MiddleScoreOneBalanceAuto extends AutoBase {
    public MiddleScoreOneBalanceAuto(
        AutoConfiguration autoConfiguration,
        AutoRequirements autoRequirements
    ) {
        super(autoConfiguration, autoRequirements);
    }
    
    public void init() {
        // Recenter offset to zero is the middle node
        final double startingYOffset = getStartingYOffsetInches(
            Grid.MIDDLE_GRID, 
            autoConfiguration.getStartingNode()
        ) + (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);

        final Pose2d initialPose = createPose2dInches(0, startingYOffset, 0);
        final Pose2d lineUpPose = createPose2dInches(24, 0, 0);
        final Pose2d chargeStationPose = createPose2dInches(134, 0, 0);
        final Pose2d driveOverPose = createPose2dInches(200, 1, 0);
        final Pose2d finalChargeStationPose = createPose2dInches(120, 0, 180);

        final AutoTrajectoryConfig retractTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 1);
        final AutoTrajectoryConfig chargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 1, 0.5);
        final AutoTrajectoryConfig driveOverTrajectoryConfig = new AutoTrajectoryConfig(0.5, 1.5, 0.5, 3, 2, 0.5, 0);
        final AutoTrajectoryConfig rechargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 0, 0);

        addCommands(new ResetOdometryCommand(autoRequirements.getDrivetrain(), initialPose));
        
        // Initial elevator score command.
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm()));
        } else {
            addCommands(new TopScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm()));
        }
        addCommands(new InstantCommand(() -> autoRequirements.getIntake().setScoreMode(autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? ScoreMode.CUBE : ScoreMode.CONE)));
        addCommands(new ScoreCommand(() -> autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.25, autoRequirements.getIntake()));
        
        SwerveControllerCommand lineUpPath = createSwerveCommand(
            retractTrajectoryConfig, 
            initialPose,
            lineUpPose,
            createRotation(180)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            lineUpPath.beforeStarting(new WaitCommand(0.5)),
            new CompleteScoreCommand(autoRequirements.getElevator(), autoRequirements.getIntake(), autoRequirements.getArm())
        );
        addCommands(retractGroup);

        SwerveControllerCommand onChargePath = createSwerveCommand(
            chargeStationTrajectoryConfig, 
            getLastEndingPose(),
            chargeStationPose,
            createRotation(180)
        );
        ParallelDeadlineGroup onChargeGroup = new ParallelDeadlineGroup(
            onChargePath,
            new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, autoRequirements.getElevator())
        );
        addCommands(onChargeGroup);

        SwerveControllerCommand overChargePath = createSwerveCommand(
            driveOverTrajectoryConfig, 
            getLastEndingPose(),
            driveOverPose,
            createRotation(0)
        );
        addCommands(overChargePath);

        if (autoConfiguration.getChargeStation() == ChargeStation.BALANCE) {
            SwerveControllerCommand chargeStationPath = createSwerveCommand(
                rechargeStationTrajectoryConfig, 
                getLastEndingPose(),
                finalChargeStationPose,
                createRotation(0)
            );
            addCommands(chargeStationPath);

            addCommands(new ChargeStationBalanceCommand(autoRequirements.getDrivetrain()));
        }
    }
}
