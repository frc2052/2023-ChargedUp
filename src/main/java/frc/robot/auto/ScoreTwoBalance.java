// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    Rotation2d rotation = null;
    Pose2d endPose = null;
    Pose2d startPose = null;
    SwerveControllerCommand commandOne = super.createSwerveCommand(startPose, endPose, rotation);
    
    this.addCommands(commandOne);

  }

  @Override
  public void init() {
    // TODO Auto-generated method stub
    
  }
}
