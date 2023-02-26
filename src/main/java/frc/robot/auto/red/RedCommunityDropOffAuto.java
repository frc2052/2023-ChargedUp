// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.red;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class RedCommunityDropOffAuto extends AutoBase {
    private final Grid startGrid;
    private final Node startNode;

    public RedCommunityDropOffAuto (
        Grid startGrid,
        Node startNode,    
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        ArmSubsystem arm,
        IntakeSubsystem intake
    ){
        super(drivetrain, elevator, intake, arm);

        this.startGrid = startGrid;
        this.startNode = startNode;
    }

    public void init() {
        double flip = startGrid == Grid.LEFT_GRID ? -1.0 : 1.0;

        // Score
        Pose2d initialPose = createPose2dInches(0, getLeftStartingYOffsetInches(startNode) * flip, 0);
        Translation2d chargeStationMidpoint = createTranslation2dInches(18, 6 * flip);
        // First cone pickup
        Pose2d startPickUpPose = createPose2dInches(64, 4 * flip, 0);
        Pose2d firstPickUpPose = createPose2dInches(194, 16 * flip, 0);
        // Second cone pickup
        Pose2d secondPickUpPose = createPose2dInches(195, -46, 0);
        // Third cone pickup
        Pose2d thirdPickUpPose = createPose2dInches(195, -94, 0);
        // Drop off
        Pose2d dropOffPose = createPose2dInches(108, 2 * flip, 0);

        drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));

        // Score first time
        if (startNode == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }

        SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(2), 
            initialPose,
            List.of(chargeStationMidpoint),
            startPickUpPose,
            createRotation(0)
        );

        ParallelCommandGroup retractGroup = new ParallelCommandGroup(
            new ScoreCommand(intake, arm, elevator, startNode == Node.MIDDLE_CUBE).withTimeout(
                startNode == Node.MIDDLE_CUBE ? 0 : 0.5
            ),
            backupPath.beforeStarting(new WaitCommand(0.5))
        );
        addCommands(retractGroup);

        // Drive and pickup first cone
        SwerveControllerCommand firstPickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withStartVelocity(2), 
            getLastEndingPose(), 
            firstPickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup firstPickUpGroup = new ParallelDeadlineGroup(
            firstPickupPath,
            new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
            new ArmOutCommand(arm),
            new IntakeInCommand(intake)
        );
        addCommands(firstPickUpGroup);

        SwerveControllerCommand firstDropOffPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(), 
            dropOffPose,
            createRotation(180)
        );

        ParallelDeadlineGroup firstDropOffGroup = new ParallelDeadlineGroup(
            firstDropOffPath,
            new IntakeOutCommand(intake).beforeStarting(new WaitCommand(1))
        );
        addCommands(firstDropOffGroup);

        // Drive and pickup second cone
        SwerveControllerCommand secondPickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(), 
            secondPickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup secondPickUpGroup = new ParallelDeadlineGroup(
            secondPickupPath,
            new ArmOutCommand(arm),
            new IntakeInCommand(intake)
        );
        addCommands(secondPickUpGroup);

        SwerveControllerCommand secondDropOffPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(), 
            dropOffPose,
            createRotation(180)
        );

        ParallelDeadlineGroup secondDropOffGroup = new ParallelDeadlineGroup(
            secondDropOffPath,
            new IntakeOutCommand(intake).beforeStarting(new WaitCommand(1))
        );
        addCommands(secondDropOffGroup);

        // Drive and pickup third cone
        SwerveControllerCommand thirdPickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(), 
            thirdPickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup thirdPickUpGroup = new ParallelDeadlineGroup(
            thirdPickupPath,
            new ArmOutCommand(arm),
            new IntakeInCommand(intake)
        );
        addCommands(thirdPickUpGroup);

        SwerveControllerCommand thirdDropOffPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(), 
            dropOffPose,
            createRotation(180)
        );

        ParallelDeadlineGroup thirdDropOffGroup = new ParallelDeadlineGroup(
            thirdDropOffPath,
            new IntakeOutCommand(intake).beforeStarting(new WaitCommand(1))
        );
        addCommands(thirdDropOffGroup);
    }
}