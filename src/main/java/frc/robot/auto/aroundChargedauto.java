// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class aroundChargedauto extends SequentialCommandGroup {
  private DrivetrainSubsystem drivetrian;

  /** Creates a new aroundChargedauto. */
  public aroundChargedauto(Drivetrain drivetrain) {
   

    Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
    Pose2d firstCubePos = new Pose2d(Units.inchesToMeters(122), Units.inchesToMeters(0), Rotation2d.fromDegrees(0));
    
    
    


      addCommands(
 
      );
  }
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    

  }


