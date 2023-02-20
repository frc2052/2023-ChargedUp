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
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedLeftScoreTwoAuto extends AutoBase{

/*Score gamepiece, move and rotate to gamepiece, move and rotate (strafe) to grid, 
shoot gamepiece (w/o stopping), go to chargestation */

  /** Creates a new scoretwoandbalence. */
  public RedLeftScoreTwoAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, IntakeSubsystem intake, ArmSubsystem arm) {
    super(drivetrain, elevator, intake, arm);

  
    // scoring the first game piece 
     this.addCommands(new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, this.elevator));
    this.addCommands(new ArmOutCommand(this.arm));
    this.addCommands(new WaitCommand(1.5));
    this.addCommands(new IntakeOutCommand(this.intake).withTimeout(1));
    
  drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
    //first score
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(Units.inchesToMeters(36), 0, Rotation2d.fromDegrees(0));
    SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.slowTrajectoryConfig, 
        startPose, 
        endPose,
        createRotation(0)
    );
    //retract arm while driving
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
        AutoTrajectoryConfig.slowTrajectoryConfig, 
        startPose, 
        endPose,
        createRotation(0)
    );
    //driving back to grid
    startPose = endPose;
    List<Translation2d> midpoint = List.of(new Translation2d(Units.inchesToMeters(40),Units.inchesToMeters(-5)));
    endPose = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-44), Rotation2d.fromDegrees(90));
    SwerveControllerCommand driveBackPath = super.createSwerveCommand(startPose, midpoint, endPose, createRotation(180));
 
    ParallelDeadlineGroup pickupAndCarryGroup = new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            // Drive back
            new ParallelCommandGroup(
                pickupPath, //deadline
                new ArmOutCommand(arm)
            ),
            // drive back
            new ParallelCommandGroup(
                driveBackPath, //deadline
                new ElevatorPositionCommand(ElevatorPosition.FLOOR_CONE, elevator)
            )
        ),
        new IntakeInCommand(intake).beforeStarting(new WaitCommand(3))
    );

    //pick up cone
    this.addCommands(pickupAndCarryGroup);

    //stop picking up
    ParallelDeadlineGroup carryGroup = new ParallelDeadlineGroup(
        driveBackPath, //deadline
        new ElevatorPositionCommand(ElevatorPosition.STARTING, this.elevator),
        new ArmInCommand(this.arm),
        new IntakeStopCommand(this.intake)
    );
    this.addCommands(carryGroup);
    //score the cone
     this.addCommands(new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, this.elevator));
    this.addCommands(new ArmOutCommand(this.arm));
    this.addCommands(new WaitCommand(1.5));
    this.addCommands(new IntakeOutCommand(this.intake).withTimeout(1));

    this.addCommands (new ArmInCommand(this.arm));
  }
    




  @Override
  public void init() {
    // TODO Auto-generated method stub
    
  }
}

