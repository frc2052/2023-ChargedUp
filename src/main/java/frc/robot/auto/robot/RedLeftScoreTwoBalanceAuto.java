// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

/**
 * Score gamepiece, move and rotate to pick up gamepiece, move and rotate (strafe) to grid, 
 * shoot gamepiece (w/o stopping), & go to chargestation.
 */
public class RedLeftScoreTwoBalanceAuto extends AutoBase{
    public RedLeftScoreTwoBalanceAuto(
        Node startNode,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(drivetrain, elevator, intake, arm);

        Pose2d initialPose = createPose2dInches(0, getLeftStartingYOffsetInches(startNode), 0);
        Translation2d chargeStationMidpoint = createTranslation2dInches(24, -2);
        Pose2d startPickUpPose = createPose2dInches(64, -4, 0);
        Pose2d approachPickUpPose = createPose2dInches(180, -8, 0);
        Pose2d pickUpPose = createPose2dInches(195, -12, 180);
        Translation2d scorePathMidpoint = createTranslation2dInches(108, -2);
        Pose2d lineUpPose = createPose2dInches(24, -66, 270);
        Pose2d chargeStationPose = createPose2dInches(100, -66, 180);
       
        drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));

        // Score first time
        if (startNode == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }

        SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.slowTrajectoryConfig.withEndVelocity(2), 
            initialPose,
            List.of(chargeStationMidpoint),
            startPickUpPose,
            createRotation(0)
        );

        ParallelCommandGroup retract = new ParallelCommandGroup(
            new ScoreCommand(intake, arm, elevator, startNode == Node.MIDDLE_CUBE).withTimeout(
                startNode == Node.MIDDLE_CUBE ? 0 : 0.5
            ),
            backupPath.beforeStarting(new WaitCommand(0.5))
        );
        
        addCommands(retract);

        // Drive to approach cone
        SwerveControllerCommand approachPickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartAndEndVelocity(2, 0.5), 
            getLastEndingPose(), 
            approachPickUpPose,
            createRotation(0)
        );

        addCommands(
            new ParallelDeadlineGroup(
                approachPickupPath,
                new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
                new ArmOutCommand(arm),
                new IntakeInCommand(intake)
            )
        );

        // Drive to pickup cone
        SwerveControllerCommand pickUpPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.slowTrajectoryConfig.withStartVelocity(0.5), 
            getLastEndingPose(), 
            pickUpPose,
            createRotation(0)
        );

        addCommands(pickUpPath);

        addCommands(new ArmInCommand(arm));

        // Driving back to grid
        SwerveControllerCommand driveBackPath = createSwerveCommand(
            getLastEndingPose(), 
            List.of(scorePathMidpoint, chargeStationMidpoint), 
            lineUpPose, 
            createRotation(180)
        );

        addCommands(
            new ParallelDeadlineGroup(
                driveBackPath,
                new IntakeOutCommand(intake).beforeStarting(new WaitCommand(2.75))
            )
        );

        addCommands(new IntakeStopCommand(intake));

        SwerveControllerCommand balancePath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.chargeStationTrajectoryConfig,
            getLastEndingPose(), 
            chargeStationPose, 
            createRotation(180)
        );

        addCommands(balancePath);
        addCommands(new RunCommand(() -> drivetrain.xWheels(), drivetrain));

        // // In front of grid
        // Pose2d endPose = new Pose2d(Units.inchesToMeters(16), Units.inchesToMeters(-67), Rotation2d.fromDegrees(0));
        // SwerveControllerCommand scorePath = super.createSwerveCommand(
        //     getLastEndingPose(), 
        //     endPose, 
        //     createRotation(180)
        // );

        // ParallelDeadlineGroup shootGroup = new ParallelDeadlineGroup(
        //     scorePath, //deadline
        //     new IntakeOutCommand(intake)
        // );
        
        // addCommands(shootGroup);

        // // turn to 0
        // Pose2d endPose = new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(-67), Rotation2d.fromDegrees(0));
        // SwerveControllerCommand spinPath = createSwerveTrajectoryCommand(
        //     AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig,
        //     getLastEndingPose(), 
        //     endPose, 
        //     createRotation(0)
        // );

        // ParallelDeadlineGroup spinGroup = new ParallelDeadlineGroup(
        //     spinPath, //deadline
        //     new ArmOutCommand(this.arm),
        //     new ElevatorPositionCommand(ElevatorPosition.FLOOR_CONE, this.elevator)
        // );

        // this.addCommands(spinGroup);

        //     //going on charge station
        //     startPose = endPose;
        //     endPose = new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(-67), Rotation2d.fromDegrees(0));
        //    SwerveControllerCommand onChargePath = super.createSwerveCommand(startPose, midpoint, endPose, createRotation(180));
        
        //         ParallelDeadlineGroup onChargeGroup = new ParallelDeadlineGroup(
        //                 onChargePath, //deadline
        //                 new IntakeStopCommand(this.intake)
                        
        //                 );  
        //     this.addCommands(onChargeGroup);
        //     this.addCommands(new ChargeStationBalanceCommand(this.drivetrain));
    }
}
