// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveDistance extends CommandBase {
  private Subsystem m_drive;
  private double m_distance;
  private double m_speed;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double meters, Subsystem drivetrain) {
    m_distance = meters;
    m_speed = speed;
    m_drive = drivetrain;
    addRequirements(drivetrain);
    
  }

  public DriveDistance(double speed, double meters, Drivetrain drivetrain) {

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arcadeDrive(0, 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.arcadeDrive(m_speed, 0);
  }

  private void arcadeDrive(double m_speed2, int j) {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.arcadeDrive(0, 0);
  }

  private void arcadeDrive(int i, int j) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(this.getAverageDistanceMeter()) >= m_distance;
  }

  private int getAverageDistanceMeter() {
    return 2;
  }
}
