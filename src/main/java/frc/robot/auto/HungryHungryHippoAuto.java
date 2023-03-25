// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

@AutoDescription(description = "Nom Nom")
public class HungryHungryHippoAuto extends ScorePickUpAutoBase {
    public HungryHungryHippoAuto (
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);
    }

    public void init() {
        super.init();

        // Second cone pickup
        Translation2d secondLineUpMidpoint = createTranslation2dInches(185, -46);
        Pose2d secondPickUpPose = createPose2dInches(195, -46, 0);
        // Third cone pickup
        Translation2d thirdLineUpMidpoint = createTranslation2dInches(185, -100);
        Pose2d thirdPickUpPose = createPose2dInches(195, -100, 0);
        // Drop off
        
        Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(120, -2);
        Pose2d dropOffPose = createPose2dInches(108, -2, 0);

        SwerveControllerCommand firstDropOffPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(), 
            dropOffPose,
            createRotation(180)
        );

        ParallelDeadlineGroup firstDropOffGroup = new ParallelDeadlineGroup(
            firstDropOffPath,
            new ArmOutCommand(arm).andThen(new IntakeOutCommand(intake).beforeStarting(new WaitCommand(1)))
        );
        addCommands(firstDropOffGroup);

        // Drive and pickup second cone
        SwerveControllerCommand secondPickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(),
            List.of(farChargeStationInterpolationPoint, secondLineUpMidpoint),
            secondPickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup secondPickUpGroup = new ParallelDeadlineGroup(
            secondPickupPath,
            new ArmOutCommand(arm).beforeStarting(new WaitCommand(0.5)).andThen(
                new ParallelCommandGroup(
                    new ElevatorPositionCommand(ElevatorPosition.GROUND_PICKUP, elevator),
                    new IntakeInCommand(intake)
                )
            )
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
            new ArmInCommand(arm).andThen(new IntakeOutCommand(intake).beforeStarting(new WaitCommand(1)))
        );
        addCommands(secondDropOffGroup);

        // Drive and pickup third cone
        SwerveControllerCommand thirdPickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            getLastEndingPose(), 
            List.of(farChargeStationInterpolationPoint, thirdLineUpMidpoint),
            thirdPickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup thirdPickUpGroup = new ParallelDeadlineGroup(
            thirdPickupPath,
            new ArmOutCommand(arm).beforeStarting(new WaitCommand(0.5)).andThen(
                new ParallelCommandGroup(
                    new ElevatorPositionCommand(ElevatorPosition.GROUND_PICKUP, elevator),
                    new IntakeInCommand(intake)
                )
            )
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
            new ArmInCommand(arm).andThen(new IntakeOutCommand(intake).beforeStarting(new WaitCommand(1)))
        );
        addCommands(thirdDropOffGroup);
    }
}