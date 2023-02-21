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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
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

    Pose2d initialPose = createPose2dInches(0, getLeftStartingYOffsetInches(startNode), 0);
    Translation2d chargeStationMidpoint = createTranslation2dInches(36, -4);
    Pose2d startPickUpPose = createPose2dInches(60, -8, 0);
    Pose2d pickUpPose = createPose2dInches(190, -8, 0);
    Pose2d lineUpPose = createPose2dInches(152, -66, 180);
    Pose2d chargeStationPose = createPose2dInches(80, -66, 180);

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

    // Drive to pickup cone
    SwerveControllerCommand pickupPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartVelocity(2), 
        getLastEndingPose(), 
        pickUpPose,
        createRotation(0)
    );

    addCommands(new ParallelDeadlineGroup(
        pickupPath,
        new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
        new ArmOutCommand(arm),
        new IntakeInCommand(intake)
    ));

    addCommands(new WaitCommand(0.25));

    // Drive to lineup w/ charge station
    SwerveControllerCommand lineupPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig, 
        getLastEndingPose(), 
        lineUpPose, 
        createRotation(180)
    );

    addCommands(new ParallelDeadlineGroup(
        lineupPath, 
        new ArmInCommand(arm),
        new IntakeStopCommand(intake).beforeStarting(new WaitCommand(2))
    ));

    //going on charge station
    SwerveControllerCommand onChargePath = super.createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
        getLastEndingPose(), 
        chargeStationPose, 
        createRotation(180)
    );

    this.addCommands(onChargePath);
    this.addCommands(new RunCommand(() -> drivetrain.xWheels(), drivetrain));
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