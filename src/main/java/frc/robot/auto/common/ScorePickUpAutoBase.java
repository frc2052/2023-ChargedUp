// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

public class ScorePickUpAutoBase extends AutoBase {
    public ScorePickUpAutoBase(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }

    @Override
    public void init() {
        final double startingYOffset = getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), 
            autoConfiguration.getStartingNode()
        );

        final Pose2d initialPose = createPose2dInches(0, startingYOffset, 0);
        final Pose2d bumpBackupPose = createPose2dInches(18, -6, 0);
        // final Translation2d chargeStationMidpoint = createTranslation2dInches(48, -4);
        final Pose2d cableProtectorPose = createPose2dInches(86, -4, 0);
        final Pose2d startPickUpPose = createPose2dInches(118, -4, 0);
        final Pose2d pickUpPose = createPose2dInches(202, -4, 0);

        final AutoTrajectoryConfig retractTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 3, 1.5, 0, 1);
        final AutoTrajectoryConfig backupTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 1, 2);
        final AutoTrajectoryConfig pickupLineUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 2, 1);
        final AutoTrajectoryConfig pickupTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 1, 0);

        addCommands(new ResetOdometryCommand(initialPose));
        
        // Initial elevator score command.
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm()));
        } else {
            addCommands(new TopScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm()));
        }
        addCommands(new InstantCommand(() -> autoRequirements.getIntake().setScoreMode(autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? ScoreMode.CUBE : ScoreMode.CONE)));
        addCommands(new ScoreCommand(() -> autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.25, autoRequirements.getIntake()));

        // Drive back slightly and retract to avoid rotation collisions with the grid.
        SwerveControllerCommand retractPath = createSwerveCommand(
            retractTrajectoryConfig, 
            initialPose, 
            bumpBackupPose,
            createRotation(45)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            retractPath, 
            new CompleteScoreCommand(autoRequirements.getElevator(), autoRequirements.getIntake(), autoRequirements.getArm())
        );
        addCommands(retractGroup);

        // Slow down over cable protector to avoid odometry drift.
        SwerveControllerCommand backupPath = createSwerveCommand(
            backupTrajectoryConfig, 
            getLastEndingPose(),
            cableProtectorPose,
            createRotation(0)
        );        
        SwerveControllerCommand pickupLineUpCommand = createSwerveCommand(
            pickupLineUpTrajectoryConfig, 
            getLastEndingPose(),
            startPickUpPose,
            createRotation(0)
        );
        ParallelDeadlineGroup pickupLineUpGroup = new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                backupPath,
                pickupLineUpCommand
            ),
            new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator())                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        );
        addCommands(pickupLineUpGroup);

        // Drive to approach and pick up the cone.
        Command pickupCommand = null;
        if (!Dashboard.getInstance().pixyCamBroken()) {
            pickupCommand = new GamePieceAlignmentCommand(
                () -> pickUpPose.getX(),
                autoRequirements.getDrivetrain(),
                autoRequirements.getForwardPixy()
            );
            setLastEndingPose(pickUpPose);
        } else {
            pickupCommand = createSwerveCommand(
                pickupTrajectoryConfig, 
                getLastEndingPose(),
                pickUpPose,
                createRotation(0)
            );
        }
        ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
            pickupCommand,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator()),
            new ArmOutCommand(autoRequirements.getArm()),
            new IntakeInCommand(autoRequirements.getIntake())
        );
        addCommands(pickUpGroup);

        // Retract arm for the start of the next path.
        addCommands(new ArmInCommand(autoRequirements.getArm()));
    }
}
