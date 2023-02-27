// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RedCommunityDropOffAuto extends ScorePickUpAuto {
    public RedCommunityDropOffAuto (
        Grid startGrid,
        Node startNode,
        boolean endChargeStation,  
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        ArmSubsystem arm,
        IntakeSubsystem intake
    ){
        super(startGrid, startNode, endChargeStation, drivetrain, elevator, intake, arm);
    }

    public void init() {
        super.init();

        double flip = startGrid == Grid.LEFT_GRID ? -1.0 : 1.0;

        // Second cone pickup
        Pose2d secondPickUpPose = createPose2dInches(195, 46 * flip, 0);
        // Third cone pickup
        Pose2d thirdPickUpPose = createPose2dInches(195, 94 * flip, 0);
        // Drop off
        Pose2d dropOffPose = createPose2dInches(108, 2 * flip, 0);

        addCommands(new ArmOutCommand(arm));

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