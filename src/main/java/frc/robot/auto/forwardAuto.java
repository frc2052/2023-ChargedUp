// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.sql.Time;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class forwardAuto extends CommandBase {
  private DrivetrainSubsystem driveTrain;
  

  /** Creates a new coneAuto. */
  public forwardAuto(DrivetrainSubsystem driveTrain) {
   
  this.driveTrain = driveTrain;
  addRequirements(driveTrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  
  

 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addCommands(
      new RunCommand(
        () -> driveTrain.drive(0.25, 0.25), 
        driveTrain 
      ).withTimeout(2)
    );
  }

  private void addCommands(ParallelRaceGroup withTimeout) {
  }






  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
