// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoBalance extends AutoBase{

/*Score gamepiece, move and rotate to gamepiece, move and rotate (strafe) to grid, 
shoot gamepiece (w/o stopping), go to chargestation */

  /** Creates a new scoretwoandbalence. */
  public ScoreTwoBalance(DrivetrainSubsystem drivetrain) {
    super(drivetrain);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Drive to pick up first cone
    Rotation2d rotation = Rotation2d.fromDegrees(180);
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    Pose2d endPose = new Pose2d(87, -8, Rotation2d.fromDegrees(0));
    SwerveControllerCommand commandOne = super.createSwerveCommand(startPose, endPose, rotation);    
    this.addCommands(commandOne);


    //driving back to grid
    rotation = Rotation2d.fromDegrees(180);
    startPose = endPose;
    List<Translation2d> midpoint = List.of(new Translation2d(40,-5));
    endPose = new Pose2d(6, -33, Rotation2d.fromDegrees(90));
   SwerveControllerCommand commandTwo = super.createSwerveCommand(startPose, midpoint, endPose, rotation);
    this.addCommands(commandTwo);

    //driving sideways in front of grid
    rotation = Rotation2d.fromDegrees(180);
    startPose = endPose;
    midpoint = List.of(new Translation2d(6,-55));
    endPose = new Pose2d(30, 67, Rotation2d.fromDegrees(0));
   SwerveControllerCommand commandThree = super.createSwerveCommand(startPose, midpoint, endPose, rotation);
    this.addCommands(commandThree);

    //going to charge station
     rotation = Rotation2d.fromDegrees(180);
    startPose = endPose;
    endPose = new Pose2d(76, 67, Rotation2d.fromDegrees(0));
   SwerveControllerCommand commandFour = super.createSwerveCommand(startPose, midpoint, endPose, rotation);
    this.addCommands(commandFour);


  }
 
  @Override
  protected void init() {
    // TODO Auto-generated method stub
    
  }
}
