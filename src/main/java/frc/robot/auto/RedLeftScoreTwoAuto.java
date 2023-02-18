// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
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
    ElevatorPositionCommand top = new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, this.elevator);
    this.addCommands(top.until(() -> elevator.atPosition()));
    this.addCommands(new ArmOutCommand(this.arm).withTimeout(1));
    this.addCommands(new IntakeInCommand(this.intake).withTimeout(1));

    //Drive to pick up first cone
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    Pose2d endPose = new Pose2d(Units.inchesToMeters(175), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
    SwerveControllerCommand commandOne = super.createSwerveCommand(startPose, endPose, createRotation(180));    
    this.addCommands(commandOne);

    //drive to pickup cone
    startPose = endPose;
    endPose = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
    SwerveControllerCommand pickupPath = super.createSwerveCommand(startPose, endPose, createRotation(0));

    //pickup cone
    ParallelDeadlineGroup pickupGroup = new ParallelDeadlineGroup(
                pickupPath, //deadline
                new ElevatorPositionCommand(ElevatorPosition.FLOOR_CONE, this.elevator),
                new ArmOutCommand(this.arm),
                new IntakeInCommand(this.intake)
                );
    this.addCommands(pickupGroup);

    //driving back to grid
    startPose = endPose;
    List<Translation2d> midpoint = List.of(new Translation2d(Units.inchesToMeters(40),Units.inchesToMeters(-5)));
    endPose = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-44), Rotation2d.fromDegrees(90));
    SwerveControllerCommand driveBackPath = super.createSwerveCommand(startPose, midpoint, endPose, createRotation(180));

    //stop picking up
    ParallelDeadlineGroup carryGroup = new ParallelDeadlineGroup(
                driveBackPath, //deadline
                new ElevatorPositionCommand(ElevatorPosition.STARTING, this.elevator),
                new ArmInCommand(this.arm),
                new IntakeStopCommand(this.intake)
                );
    this.addCommands(carryGroup);

    ElevatorPositionCommand top2 = new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, this.elevator);
    this.addCommands(top2.until(() -> elevator.atPosition()));
    this.addCommands(new ArmOutCommand(this.arm).withTimeout(1));
    this.addCommands(new IntakeInCommand(this.intake).withTimeout(1));
  }
    




  @Override
  public void init() {
    // TODO Auto-generated method stub
    
  }
}

