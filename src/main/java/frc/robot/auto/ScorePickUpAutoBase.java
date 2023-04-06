// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;
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
        Pose2d tinyBackupPose2d = createPose2dInches(18, -6, 0);
        Translation2d chargeStationMidpoint = createTranslation2dInches(48, -4);
        Pose2d startPickUpPose = createPose2dInches(112, -6, 0);
        Pose2d pickUpPose = createPose2dInches(194, -14, 0);

        addCommands(new ResetOdometryCommand(drivetrain, initialPose));

        // Score first time
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }

        addCommands(new ScoreCommand(() -> ScoreMode.CONE, intake, arm, elevator).withTimeout(
            autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.5
        ));

        // Drive back slightly to avoid rotation collisions with the grid.
        SwerveControllerCommand tinyBackupCommand = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withEndVelocity(1), 
            initialPose, 
            tinyBackupPose2d,
            createRotation(90)
        );
        addCommands(tinyBackupCommand);

        // Slow down over cable protector to avoid odometry drift.
        SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withStartAndEndVelocity(1, 2), 
            getLastEndingPose(),
            List.of(chargeStationMidpoint),
            startPickUpPose,
            createRotation(0)
        );
        // addCommands(backupPath);

        ParallelDeadlineGroup backUpGroup = new ParallelDeadlineGroup(
            backupPath,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_PICKUP, elevator)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        );
        addCommands(backUpGroup);

        // Drive to approach and pick up the cone.
        SwerveControllerCommand pickUpPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withStartVelocity(2), 
            getLastEndingPose(),
            pickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
            pickUpPath,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_PICKUP, elevator),
            new ArmOutCommand(arm),
            new IntakeInCommand(intake)
        );

        addCommands(pickUpGroup);

        addCommands(new ArmInCommand(arm));

        // Pose2d initialPose = createPose2dInches(0, getLeftStartingYOffsetInches(startGrid, startNode) * flip, 0);
        // Translation2d chargeStationMidpoint = createTranslation2dInches(18, 6 * flip);
        // Pose2d startPickUpPose = createPose2dInches(64, 4 * flip, 0);
        // Pose2d pickUpPose = createPose2dInches(194, 12 * flip, 0);

        // drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));

        // // Score first time
        // if (startNode == Node.MIDDLE_CUBE) {
        //     addCommands(new MidScoreCommand(elevator, arm));
        // } else {
        //     addCommands(new TopScoreCommand(elevator, arm));
        // }

        // SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
        //     AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(2), 
        //     initialPose,
        //     List.of(chargeStationMidpoint),
        //     startPickUpPose,
        //     createRotation(0)
        // );

        // ParallelCommandGroup retractGroup = new ParallelCommandGroup(
        //     new ScoreCommand(intake, arm, elevator, startNode == Node.MIDDLE_CUBE).withTimeout(
        //         startNode == Node.MIDDLE_CUBE ? 0 : 0.5
        //     ),
        //     backupPath.beforeStarting(new WaitCommand(0.5))
        // );
        // addCommands(retractGroup);

        // // Drive to approach cone
        // SwerveControllerCommand pickupPath = createSwerveTrajectoryCommand(
        //     AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartVelocity(2), 
        //     getLastEndingPose(), 
        //     pickUpPose,
        //     createRotation(0)
        // );

        // ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
        //     pickupPath,
        //     new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
        //     new ArmOutCommand(arm),
        //     new IntakeInCommand(intake)
        // );
        // addCommands(pickUpGroup);
        // addCommands(new ArmInCommand(arm));
    }
}
