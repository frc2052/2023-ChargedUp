// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.robot.red;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.MatchEndBlockCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.NewChargeStationBalanceCommand;
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
public class RedRightScoreTwoBalanceAuto extends AutoBase{
    public RedRightScoreTwoBalanceAuto(
        Node startNode,
        boolean endChargeStation,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(drivetrain, elevator, intake, arm);

        Pose2d initialPose = createPose2dInches(0, getLeftStartingYOffsetInches(startNode), 0);
        Translation2d chargeStationMidpoint = createTranslation2dInches(24, -2);
        Pose2d startPickUpPose = createPose2dInches(64, -4, 0);
        Pose2d pickUpPose = createPose2dInches(194, -16, 0);
        Translation2d scorePathMidpoint = createTranslation2dInches(108, -2);
        Pose2d lineUpPose = createPose2dInches(24, -80, 270);
        Pose2d chargeStationPose = createPose2dInches(100, -80, 180);
       
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
            backupPath.beforeStarting(new WaitCommand(startNode == Node.MIDDLE_CUBE ? 0 : 0.5))
        );
        
        addCommands(retract);

        // Drive to approach cone
        SwerveControllerCommand pickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartVelocity(2), 
            getLastEndingPose(), 
            pickUpPose,
            createRotation(0)
        );

        addCommands(
            new ParallelDeadlineGroup(
                pickupPath,
                new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
                new ArmOutCommand(arm),
                new IntakeInCommand(intake)
            )
        );

        addCommands(new ArmInCommand(arm));

        // Driving back to grid
        SwerveControllerCommand driveBackPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.slowTrajectoryConfig.withEndVelocity(2),
            getLastEndingPose(), 
            List.of(scorePathMidpoint, chargeStationMidpoint), 
            lineUpPose, 
            createRotation(180)
        );

        addCommands(
            new ParallelDeadlineGroup(
                driveBackPath,
                new IntakeOutCommand(intake).beforeStarting(new WaitCommand(4))
            )
        );

        addCommands(new IntakeStopCommand(intake));

        if (endChargeStation) {
            SwerveControllerCommand balancePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig.withStartVelocity(2),
                getLastEndingPose(), 
                chargeStationPose, 
                createRotation(180)
            );
    
            addCommands(balancePath);

            addCommands(
                new ParallelDeadlineGroup(
                    new MatchEndBlockCommand(),
                    new NewChargeStationBalanceCommand(drivetrain)
                )
            );

            addCommands(new RunCommand(() -> { drivetrain.xWheels(); }, drivetrain));
        }
    }
}
