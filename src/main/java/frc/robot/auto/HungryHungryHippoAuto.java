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
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.ScorePickUpAutoBase;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

@AutoDescription(description = "Nom Nom")
@DashboardAutoRequirements(requirements = { Grid.class, Node.class })
public class HungryHungryHippoAuto extends ScorePickUpAutoBase {
    public HungryHungryHippoAuto (
        AutoConfiguration autoConfiguration,
        AutoRequirements autoRequirements
    ) {
        super(autoConfiguration, autoRequirements);
    }

    public void init() {
        super.init();

        final Translation2d secondLineUpMidpoint = createTranslation2dInches(185, -46);
        final Pose2d secondPickUpPose = createPose2dInches(195, -46, 0);
        final Translation2d thirdLineUpMidpoint = createTranslation2dInches(185, -100);
        final Pose2d thirdPickUpPose = createPose2dInches(195, -100, 0);
        final Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(120, -2);
        final Pose2d dropOffPose = createPose2dInches(108, -2, 0);

        final AutoTrajectoryConfig firstDropOffTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 0);
        final AutoTrajectoryConfig secondPickUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 0);
        final AutoTrajectoryConfig secondDropOffTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 0);
        final AutoTrajectoryConfig thirdPickUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 0);
        final AutoTrajectoryConfig thirdDropOffTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 0);

        SwerveControllerCommand firstDropOffPath = createSwerveCommand(
            firstDropOffTrajectoryConfig, 
            getLastEndingPose(), 
            dropOffPose,
            createRotation(180)
        );

        ParallelDeadlineGroup firstDropOffGroup = new ParallelDeadlineGroup(
            firstDropOffPath,
            new ArmOutCommand(autoRequirements.getArm()).andThen(new IntakeOutCommand(autoRequirements.getIntake()).beforeStarting(new WaitCommand(1)))
        );
        addCommands(firstDropOffGroup);

        // Drive and pickUp second cone
        SwerveControllerCommand secondPickUpPath = createSwerveCommand(
            secondPickUpTrajectoryConfig, 
            getLastEndingPose(),
            List.of(farChargeStationInterpolationPoint, secondLineUpMidpoint),
            secondPickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup secondPickUpGroup = new ParallelDeadlineGroup(
            secondPickUpPath,
            new ArmOutCommand(autoRequirements.getArm()).beforeStarting(new WaitCommand(0.5)).andThen(
                new ParallelCommandGroup(
                    new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator()),
                    new IntakeInCommand(autoRequirements.getIntake())
                )
            )
        );
        addCommands(secondPickUpGroup);

        SwerveControllerCommand secondDropOffPath = createSwerveCommand(
            secondDropOffTrajectoryConfig, 
            getLastEndingPose(), 
            dropOffPose,
            createRotation(180)
        );

        ParallelDeadlineGroup secondDropOffGroup = new ParallelDeadlineGroup(
            secondDropOffPath,
            new ArmInCommand(autoRequirements.getArm()).andThen(new IntakeOutCommand(autoRequirements.getIntake()).beforeStarting(new WaitCommand(1)))
        );
        addCommands(secondDropOffGroup);

        // Drive and pickUp third cone
        SwerveControllerCommand thirdPickUpPath = createSwerveCommand(
            thirdPickUpTrajectoryConfig, 
            getLastEndingPose(), 
            List.of(farChargeStationInterpolationPoint, thirdLineUpMidpoint),
            thirdPickUpPose,
            createRotation(0)
        );

        ParallelDeadlineGroup thirdPickUpGroup = new ParallelDeadlineGroup(
            thirdPickUpPath,
            new ArmOutCommand(autoRequirements.getArm()).beforeStarting(new WaitCommand(0.5)).andThen(
                new ParallelCommandGroup(
                    new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator()),
                    new IntakeInCommand(autoRequirements.getIntake())
                )
            )
        );
        addCommands(thirdPickUpGroup);

        SwerveControllerCommand thirdDropOffPath = createSwerveCommand(
            thirdDropOffTrajectoryConfig, 
            getLastEndingPose(), 
            dropOffPose,
            createRotation(180)
        );

        ParallelDeadlineGroup thirdDropOffGroup = new ParallelDeadlineGroup(
            thirdDropOffPath,
            new ArmInCommand(autoRequirements.getArm()).andThen(new IntakeOutCommand(autoRequirements.getIntake()).beforeStarting(new WaitCommand(1)))
        );
        addCommands(thirdDropOffGroup);
    }
}