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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedLeftScoreOneBalanceAuto extends AutoBase {

/*Score gamepiece, move and rotate to gamepiece, move and rotate (strafe) to grid, 
shoot gamepiece (w/o stopping), go to chargestation */

  /** Creates a new scoretwoandbalence. */
  public RedLeftScoreOneBalanceAuto(
    Node startNode,
    DrivetrainSubsystem drivetrain, 
    ElevatorSubsystem elevator, 
    IntakeSubsystem intake, 
    ArmSubsystem arm
) {
    super(drivetrain, elevator, intake, arm);

    // Score first time
    this.addCommands(new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, this.elevator));
    this.addCommands(new ArmOutCommand(this.arm));
    this.addCommands(new WaitCommand(1.5));
    this.addCommands(new IntakeOutCommand(this.intake).withTimeout(1));

    drivetrain.resetOdometry(new Pose2d(0, super.getLeftStartingYOffsetMeters(startNode), Rotation2d.fromDegrees(180)));

    Pose2d startPose = new Pose2d(0, super.getLeftStartingYOffsetMeters(startNode), Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(Units.inchesToMeters(100), 0, Rotation2d.fromDegrees(0));
    SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.slowTrajectoryConfig.withEndVelocity(2), 
        startPose,
        List.of(new Translation2d(Units.inchesToMeters(36), Units.inchesToMeters(-6))),
        endPose,
        createRotation(0)
    );

    ParallelCommandGroup retract = new ParallelCommandGroup(
        backupPath,
        new ArmInCommand(this.arm),
        new ElevatorPositionCommand(ElevatorPosition.STARTING, this.elevator).beforeStarting(new WaitCommand(0.5)),
        new IntakeStopCommand(this.intake)
    );
    
    addCommands(retract);

    //drive to pickup cone
    startPose = endPose;
    endPose = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
    SwerveControllerCommand pickupPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.slowTrajectoryConfig.withStartVelocity(2), 
        startPose, 
        endPose,
        createRotation(0)
    );

    //drive to lineup w/ charge station
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
                pickupPath, //deadline
                new ArmOutCommand(arm)
            ),
            // Line up command
            new ParallelCommandGroup(
                lineupPath, //deadline
                //new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
                new ArmInCommand(arm)
            )
        ),
        new IntakeInCommand(intake).beforeStarting(new WaitCommand(3))
    );

    //pickup cone
    this.addCommands(pickupAndCarryGroup);

    addCommands(new ArmInCommand(arm));

    //going on charge station
    startPose = endPose;
    endPose = new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(-66), Rotation2d.fromDegrees(180));
    SwerveControllerCommand onChargePath = super.createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
        startPose, 
        endPose, 
        createRotation(180)
    );

    // this.addCommands(onChargePath);
    // this.addCommands(new ChargeStationBalanceCommand(this.drivetrain));

    // // Score first time
    // this.addCommands(new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, this.elevator));
    // this.addCommands(new ArmOutCommand(this.arm));
    // this.addCommands(new WaitCommand(1.5));
    // this.addCommands(new IntakeOutCommand(this.intake).withTimeout(1));

    // drivetrain.resetOdometry(new Pose2d(0, getLeftStartingYOffsetMeters(startNode), Rotation2d.fromDegrees(180)));

    // Pose2d startPose = new Pose2d(0, getLeftStartingYOffsetMeters(startNode), Rotation2d.fromDegrees(0));
    // Pose2d endPose = new Pose2d(Units.inchesToMeters(100), 0, Rotation2d.fromDegrees(0));
    // SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
    //     AutoTrajectoryConfig.slowTrajectoryConfig.withEndVelocity(2), 
    //     startPose,
    //     List.of(new Translation2d(Units.inchesToMeters(36), 0)),
    //     endPose,
    //     createRotation(0)
    // );

    // ParallelCommandGroup retract = new ParallelCommandGroup(
    //     backupPath,
    //     new ArmInCommand(this.arm),
    //     new ElevatorPositionCommand(ElevatorPosition.STARTING, this.elevator).beforeStarting(new WaitCommand(0.5)),
    //     new IntakeStopCommand(this.intake)
    // );
    
    // addCommands(retract);

    // //drive to pickup cone
    // startPose = endPose;
    // endPose = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
    // SwerveControllerCommand pickupPath = createSwerveTrajectoryCommand(
    //     AutoTrajectoryConfig.slowTrajectoryConfig.withStartVelocity(2), 
    //     startPose, 
    //     endPose,
    //     createRotation(0)
    // );

    // //drive to lineup w/ charge station
    // startPose = endPose;
    // endPose = new Pose2d(Units.inchesToMeters(142), Units.inchesToMeters(-66), Rotation2d.fromDegrees(180));
    // SwerveControllerCommand lineupPath = createSwerveTrajectoryCommand(
    //     AutoTrajectoryConfig.slowTrajectoryConfig, 
    //     startPose, 
    //     endPose, 
    //     createRotation(180)
    // );

    // addCommands(pickupPath);
    // addCommands(lineupPath);

    // //new IntakeInCommand(intake);

    // ParallelDeadlineGroup pickupAndCarryGroup = new ParallelDeadlineGroup(
    //     new SequentialCommandGroup(
    //         // pickupPath,
    //         // lineupPath
    //         // // Drive to pick up cone command
    //         // new ParallelCommandGroup(
    //         //     pickupPath, //deadline
    //         //     new ArmOutCommand(arm)
    //         // ),
    //         // // Line up command
    //         // new ParallelCommandGroup(
    //         //     lineupPath, //deadline
    //         //     new ArmInCommand(arm)
    //         // )
    //     ),
    //     new IntakeInCommand(intake)
    // );

    // //pickup cone
    // //this.addCommands(pickupAndCarryGroup);

    // //going on charge station
    // // startPose = endPose;
    // // endPose = new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(-66), Rotation2d.fromDegrees(180));
    // // SwerveControllerCommand onChargePath = super.createSwerveTrajectoryCommand(
    // //     AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
    // //     startPose, 
    // //     endPose, 
    // //     createRotation(180)
    // // );

    // // this.addCommands(onChargePath);
    // // this.addCommands(new ChargeStationBalanceCommand(this.drivetrain));
  }


}