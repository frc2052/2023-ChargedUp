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
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.IntakeArmInCommand;
import frc.robot.commands.IntakeArmOutCommand;
import frc.robot.commands.IntakeInCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.PIDChargeStationAutoBalCommand;
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
  public RedLeftScoreOneBalanceAuto(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, IntakeSubsystem intake) {
    super(drivetrain, elevator, intake);

    AutoTrajectoryConfig drivingToTerminalTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 3.5, 2);
    AutoTrajectoryConfig intakingBothTerminalBallsTrajectoryConfig = super.createTrajectoryConfig(2, 1, 1, 3, 2); 
    AutoTrajectoryConfig backToShootTrajectoryConfig = super.createTrajectoryConfig(4, 3, 1, 3, 2); 
    AutoTrajectoryConfig driveToBall4PosTrajectoryConfig = super.createTrajectoryConfig(2, 1, 1, 3, 2);



    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //score first time
    ElevatorPositionCommand top = new ElevatorPositionCommand(ElevatorPosition.TOPSCORE, this.elevator);
    this.addCommands(top.until(() -> elevator.atPosition()));
    this.addCommands(new IntakeArmOutCommand(this.intake).withTimeout(1));
    this.addCommands(new IntakeInCommand(this.intake).withTimeout(1));

    //Drive to pick up first cone
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    Pose2d endPose = new Pose2d(Units.inchesToMeters(175), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
    SwerveControllerCommand commandOne = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, startPose, endPose, super.createRotationAngle(180));    
    this.addCommands(commandOne);

    //drive to pickup cone
    startPose = endPose;
    endPose = new Pose2d(Units.inchesToMeters(190), Units.inchesToMeters(-8), Rotation2d.fromDegrees(0));
    SwerveControllerCommand pickupPath = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPose, endPose);

    //pickup cone
    ParallelDeadlineGroup pickupGroup = new ParallelDeadlineGroup(
                pickupPath, //deadline
                new ElevatorPositionCommand(ElevatorPosition.FLOORCONE, this.elevator),
                new IntakeArmOutCommand(this.intake),
                new IntakeInCommand(this.intake)
                );
    this.addCommands(pickupGroup);

    //drive to lineup w/ charge station
    startPose = endPose;
    endPose = new Pose2d(Units.inchesToMeters(123), Units.inchesToMeters(-75), Rotation2d.fromDegrees(90));
    SwerveControllerCommand lineupPath = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPose, endPose, super.createRotationAngle(180));

    //stop picking up
    ParallelDeadlineGroup carryGroup = new ParallelDeadlineGroup(
                lineupPath, //deadline
                new ElevatorPositionCommand(ElevatorPosition.STARTING, this.elevator),
                new IntakeArmInCommand(this.intake),
                new IntakeStopCommand(this.intake)
                );
    this.addCommands(carryGroup);

    //going on charge station
    startPose = endPose;
    endPose = new Pose2d(Units.inchesToMeters(76), Units.inchesToMeters(-75), Rotation2d.fromDegrees(0));
   SwerveControllerCommand onChargePath = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPose, endPose, super.createRotationAngle(180));
   
        ParallelDeadlineGroup lineupGroup = new ParallelDeadlineGroup(
                onChargePath, //deadline
                new IntakeStopCommand(this.intake)
                
                );  
    this.addCommands(lineupGroup);
    this.addCommands(new PIDChargeStationAutoBalCommand(this.drivetrain));
  }
 
  @Override
  protected void init() {
    // TODO Auto-generated method stub
    
  }
}
