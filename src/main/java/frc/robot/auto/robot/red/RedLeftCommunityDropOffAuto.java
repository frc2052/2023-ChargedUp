// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.robot.red;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
public class RedLeftCommunityDropOffAuto extends AutoBase {
  /** Creates a new  CommunityDropOffAuto. */
  public RedLeftCommunityDropOffAuto (
  Node startNode,    
  DrivetrainSubsystem drivetrain, 
  ElevatorSubsystem elevator, 
  ArmSubsystem arm,
  IntakeSubsystem intake
  
  ){
    super(drivetrain, elevator, intake, arm);
    //begining 
    Pose2d initialPose = createPose2dInches(0, getLeftStartingYOffsetInches(startNode), 0);
    Translation2d chargeStationMidpoint = createTranslation2dInches(24, -12);
    Pose2d startPickUpPose = createPose2dInches(64, -4, 0);
    
    //first cone pickup
    Pose2d approachFirstCone = createPose2dInches(180, -12, 0);
    Pose2d pickUpFirst = createPose2dInches(195, -12, 0);
    //second cone pickup
    Pose2d approachSecondCone = createPose2dInches(180, -40, 0);
    Pose2d pickUpSecond = createPose2dInches(195, -46, 0);
    //third cone picku[]
    Pose2d approachThirdCone = createPose2dInches(180, -88, 0);
    Pose2d pickUpThird = createPose2dInches(195, -94, 0);
    
    //community drop off
    Pose2d returnCommunityPose = createPose2dInches(100, -12, 0);
  

    drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));

    // Score first time
    if (startNode == Node.MIDDLE_CUBE) {
        addCommands(new MidScoreCommand(elevator, arm));
    } else {
        addCommands(new TopScoreCommand(elevator, arm));

    }
    // backs up
    SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.slowTrajectoryConfig.withEndVelocity(2), 
        initialPose,
        List.of(chargeStationMidpoint),
        startPickUpPose,
        createRotation(0)
    );
    //takes arm and stuff back in
    ParallelCommandGroup retract = new ParallelCommandGroup(
        new ScoreCommand(intake, arm, elevator, startNode == Node.MIDDLE_CUBE).withTimeout(
            startNode == Node.MIDDLE_CUBE ? 0 : 0.5
        ),
        backupPath.beforeStarting(new WaitCommand(0.5))
    );
    
    addCommands(retract);

    //approaches first cone
    SwerveControllerCommand approachPickupPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartAndEndVelocity(2, 0.5), 
        getLastEndingPose(), 
        approachFirstCone,
        createRotation(0)
    );

    //picks up cone ?
    addCommands(new ParallelDeadlineGroup(
        approachPickupPath,
        new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
        new ArmOutCommand(arm),
        new IntakeInCommand(intake)
    ));

    //todo; make pick up first cone

     SwerveControllerCommand dropOffPath = createSwerveTrajectoryCommand(
        AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartAndEndVelocity(2, 0.5), 
        getLastEndingPose(), 
        returnCommunityPose,
        createRotation(180)
    );

       addCommands (new ParallelDeadlineGroup(
        dropOffPath,
        new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
        
        new IntakeOutCommand(intake)
        ));

        


  //   SwerveControllerCommand returnCommunityPose = createSwerveCommand(
  //       AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartAndEndVelocity(2, 0.5), 
  //       returnCommunityPose,
  //       getLastEndingPose(),
  //       createRotation(0));
  // }
  
  // addCommands(new ParallelDeadlineGroup(
  //   returnCommunityPose,
  //   new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
  //   new ArmOutCommand(arm),
  //   new IntakeInCommand(intake)
  // ));



  }
 


 







}