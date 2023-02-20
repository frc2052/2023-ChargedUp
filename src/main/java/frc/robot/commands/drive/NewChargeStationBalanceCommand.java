// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class NewChargeStationBalanceCommand extends CommandBase {

  private DrivetrainSubsystem drivetrain;
  private boolean holding;
  private Timer balanceTimer;

  private double previousPitch;
  private double currentPitch;

  /** Creates a new NewChargeStationAutoBalance. */
  public NewChargeStationBalanceCommand(
    DrivetrainSubsystem drivetrain) {

        System.out.println("NEW NewChargeStationBalanceCommand ************************* ");


    balanceTimer = new Timer();
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public void reset() {
    holding = false;
    previousPitch = 0;
    currentPitch = 0;
    balanceTimer.stop();
    balanceTimer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holding = false;
    previousPitch = 0;
    currentPitch = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute(){
    previousPitch = currentPitch;
    currentPitch = drivetrain.getNavx().getPitch();
    boolean isDropping = Math.abs(previousPitch) - Math.abs(currentPitch) > 0.6;

      if (!isDropping && !holding) {
        drivetrain.drive(
            Math.copySign(0.07, (double) -(drivetrain.getNavx().getPitch())),
            0,
            0, 
            false
        );
      } else {
          //if balance timer has gone for 2 seconds, x the wheels
          if (balanceTimer.hasElapsed(1)){
              drivetrain.xWheels();
              holding = true;
          //if the balance timer has NOT started and the bot is dropping, start timer
          } else if (!(balanceTimer.hasElapsed(0.1)) && isDropping){
              balanceTimer.start();
          } else if (!balanceTimer.hasElapsed(1)){
            //do nothing still dropping
          //if the pitch changes to greater than the tolerance, stop and reset the balance timer
          } else if ((Math.abs(drivetrain.getNavx().getPitch())) > Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES){
              balanceTimer.stop();
              balanceTimer.reset();
              holding = false;
              isDropping = false;
        }
        }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
