// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTime extends CommandBase {

  private double metersPerSecond;
  private double seconds;
  private DrivetrainSubsystem drive;
  
  /** Creates a new DriveTime. */
  public DriveTime(double metersPerSecond, double seconds, DrivetrainSubsystem drivetrain) {
    this.metersPerSecond = metersPerSecond;
    this.seconds = seconds * 1000;
    this.drive = drive;
    addRequirements(drive);
  }
  private double startTime;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    drive.drive(0.0, 0.5, 0.0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(0.0, 0.5, 0.0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0.0, 0.0, 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= seconds;
  }
}
