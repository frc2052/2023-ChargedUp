// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
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
        Pose2d initialPose = createPose2dInches(0, getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), 
            autoConfiguration.getStartingNode()
        ), 0);
        Pose2d tinyBackupPose2d = createPose2dInches(6, getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), 
            autoConfiguration.getStartingNode()
        ), 0);
        Translation2d nearChargeStationMidpoint = createTranslation2dInches(48, -4);
        Translation2d farchargeStationMidpoint = createTranslation2dInches(130, -4);
        Pose2d pickUpPose = createPose2dInches(194, -8, 0);

        addCommands(new ResetOdometryCommand(drivetrain, initialPose));

        // Score first time
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }

        addCommands(new ScoreCommand(intake, arm, elevator).withTimeout(
            autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.5
        ));

        // Drive back slightly to avoid rotation collisions with the grid.
        SwerveControllerCommand tinyBackupCommand = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(1), 
            initialPose, 
            tinyBackupPose2d,
            createRotation(90)
        );
        addCommands(tinyBackupCommand);

        // Slow down over cable protector to avoid odometry drift.
        SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withStartAndEndVelocity(1, 2.5), 
            getLastEndingPose(),
            List.of(nearChargeStationMidpoint),
            super.cableProtectorPoint,
            createRotation(0)
        );
        addCommands(backupPath);

        // Drive to approach and pick up the cone.
        SwerveControllerCommand pickUpPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withStartVelocity(2.5), 
            getLastEndingPose(),
            List.of(farchargeStationMidpoint),
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
