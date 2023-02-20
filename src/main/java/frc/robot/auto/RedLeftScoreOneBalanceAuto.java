// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

/**
 * Score gamepiece, move and rotate to pick up gamepiece, & go to chargestation.
 */
public class RedLeftScoreOneBalanceAuto extends AutoBase{
    public RedLeftScoreOneBalanceAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, IntakeSubsystem intake, ArmSubsystem arm) {
        super(drivetrain, elevator, intake, arm);
    }
 
    @Override
    public void init() {
        // Score first time
        addCommands(new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, elevator));
        addCommands(new ArmOutCommand(arm));
        addCommands(new WaitCommand(1.5));
        addCommands(new IntakeOutCommand(intake).withTimeout(1));

        drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

        Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endPose = new Pose2d(Units.inchesToMeters(36), 0, Rotation2d.fromDegrees(0));
        SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.slowTrajectoryConfig, 
            startPose, 
            endPose,
            createRotation(0)
        );

        ParallelCommandGroup retract = new ParallelCommandGroup(
            backupPath,
            new ArmInCommand(arm),
            new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator).beforeStarting(new WaitCommand(0.5)),
            new IntakeStopCommand(intake)
        );
        
        addCommands(retract);

        // Drive to pickup cone
        startPose = endPose;
        endPose = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
        SwerveControllerCommand pickupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.slowTrajectoryConfig, 
            startPose, 
            endPose,
            createRotation(0)
        );

        // Drive to lineup w/ charge station
        startPose = endPose;
        endPose = new Pose2d(Units.inchesToMeters(142), Units.inchesToMeters(-66), Rotation2d.fromDegrees(180));
        SwerveControllerCommand lineupPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.slowTrajectoryConfig, 
            startPose, 
            endPose, 
            createRotation(180)
        );

        ParallelDeadlineGroup pickupAndCarryGroup = new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // Drive to pick up cone command
                new ParallelCommandGroup(
                    pickupPath,
                    new ArmOutCommand(arm)
                ),
                // Line up command
                new ParallelCommandGroup(
                    lineupPath,
                    new ElevatorPositionCommand(ElevatorPosition.FLOOR_CONE, elevator)
                )
            ),
            new IntakeInCommand(intake).beforeStarting(new WaitCommand(3))
        );

        //pickup cone
        addCommands(pickupAndCarryGroup);

        //going on charge station
        startPose = endPose;
        endPose = new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(-66), Rotation2d.fromDegrees(180));
        SwerveControllerCommand onChargePath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
            startPose, 
            endPose, 
            createRotation(180)
        );

        addCommands(onChargePath);
        addCommands(new ChargeStationBalanceCommand(drivetrain));
    }
}
