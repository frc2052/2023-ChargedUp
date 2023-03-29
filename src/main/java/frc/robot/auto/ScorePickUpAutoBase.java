// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class ScorePickUpAutoBase extends AutoBase {
    public ScorePickUpAutoBase(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);
    }

    @Override
    public void init() {
        Pose2d initialPose = createPose2dInches(
            0, 
            getStartingYOffsetInches(
                autoConfiguration.getStartingGrid(), 
                autoConfiguration.getStartingNode()
            ), 
            0
        );

        Pose2d tinyBackupPose2d = createPose2dInches(6, getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), 
            autoConfiguration.getStartingNode()
        ), 0);

        Translation2d chargeStationMidpoint = createTranslation2dInches(18, -10);
        Pose2d startPickUpPose = createPose2dInches(92, -14, 0);
        Pose2d pickUpPose = createPose2dInches(194, -18, 0);

        addCommands(new InstantCommand(() -> { 
            drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));
        }, drivetrain));

        // Score first time
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }

        SwerveControllerCommand tinyBackupCommand = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(1), 
            initialPose, 
            tinyBackupPose2d,
            createRotation(180)
        );

        SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withStartAndEndVelocity(1, 2), 
            getLastEndingPose(),
            List.of(chargeStationMidpoint),
            startPickUpPose,
            createRotation(0)
        );

        addCommands(new ScoreCommand(intake, arm, elevator, autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE).withTimeout(
            autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.5
        ));
        addCommands(tinyBackupCommand);
        addCommands(backupPath);

        // Drive to approach cone
        SwerveControllerCommand pickUpPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withStartVelocity(2), 
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
    }
}
