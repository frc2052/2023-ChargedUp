// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedLeftScoreOneBalanceAuto extends AutoBase{

/*Score gamepiece, move and rotate to gamepiece, move and rotate (strafe) to grid, 
shoot gamepiece (w/o stopping), go to chargestation */

  /** Creates a new scoretwoandbalence. */
  public RedLeftScoreOneBalanceAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, IntakeSubsystem intake, ArmSubsystem arm) {
    super(drivetrain, elevator, intake, arm);

    // Score first time
    this.addCommands(new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, this.elevator));
    this.addCommands(new ArmOutCommand(this.arm));
    this.addCommands(new WaitCommand(1));
    this.addCommands(new IntakeOutCommand(this.intake).withTimeout(1));

    drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(Units.inchesToMeters(36), 0, Rotation2d.fromDegrees(0));
    SwerveControllerCommand backupPath = super.createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.slowTrajectoryConfig, 
        startPose, 
        endPose,
        createRotation(0)
    );

    ParallelCommandGroup retract = new ParallelCommandGroup(
        new ArmInCommand(this.arm),
        new ElevatorPositionCommand(ElevatorPosition.STARTING, this.elevator),
        new IntakeStopCommand(this.intake),
        backupPath
    );
    
    addCommands(retract);

    // Drive to pick up first cone
//    startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
//    endPose = new Pose2d(Units.inchesToMeters(-175), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
//    SwerveControllerCommand commandOne = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPose, endPose, super.createRotationAngle(180));
//    this.addCommands(commandOne);

//     //drive to pickup cone
//     startPose = endPose;
//     endPose = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
//     SwerveControllerCommand pickupPath = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPose, endPose);

//     //pickup cone
//     ParallelDeadlineGroup pickupGroup = new ParallelDeadlineGroup(
//                 pickupPath, //deadline
//                 new ElevatorPositionCommand(ElevatorPosition.FLOOR_CONE, this.elevator),
//                 new ArmOutCommand(this.arm),
//                 new IntakeInCommand(this.intake)
//                 );
//     this.addCommands(pickupGroup);

//     //drive to lineup w/ charge station
//     startPose = endPose;
//     endPose = new Pose2d(Units.inchesToMeters(123), Units.inchesToMeters(-75), Rotation2d.fromDegrees(90));
//     SwerveControllerCommand lineupPath = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPose, endPose, super.createRotationAngle(180));

//     //stop picking up
//     ParallelDeadlineGroup carryGroup = new ParallelDeadlineGroup(
//                 lineupPath, //deadline
//                 new ElevatorPositionCommand(ElevatorPosition.STARTING, this.elevator),
//                 new ArmInCommand(this.arm),
//                 new IntakeStopCommand(this.intake)
//                 );
//     this.addCommands(carryGroup);

//     //going on charge station
//     startPose = endPose;
//     endPose = new Pose2d(Units.inchesToMeters(76), Units.inchesToMeters(-75), Rotation2d.fromDegrees(0));
//    SwerveControllerCommand onChargePath = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPose, endPose, super.createRotationAngle(180));
   
//         ParallelDeadlineGroup lineupGroup = new ParallelDeadlineGroup(
//                 onChargePath, //deadline
//                 new IntakeStopCommand(this.intake)
                
//                 );  
//     this.addCommands(lineupGroup);
//     this.addCommands(new ChargeStationBalanceCommand(this.drivetrain));
  }
 
  @Override
  public void init() {
    // TODO Auto-generated method stub
    
  }
}
