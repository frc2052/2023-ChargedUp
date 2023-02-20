// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.io.Dashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class NewChargeStationAutoBalance extends CommandBase {

  private DrivetrainSubsystem drivetrain;
  private boolean holding;
  private Timer balanceTimer;

  /** Creates a new NewChargeStationAutoBalance. */
  public NewChargeStationAutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holding = false;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute(){

      Dashboard.getInstance().putData(
        "Pitch of Robot",
        drivetrain.getNavx().getPitch()
      );

      if (Math.abs(drivetrain.getNavx().getPitch()) > Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES && !holding) {
        drivetrain.drive(
            Math.copySign(0.1, (double) -(drivetrain.getNavx().getPitch())),
            0,
            0, 
            false
        );
      } else {
          //if balance timer has gone for 2 seconds, x the wheels
          if (balanceTimer.hasElapsed(2)){
              drivetrain.xWheels();
              holding = true;
          //if the balance timer has NOT started and the pitch is above the tolerance start the balance timer
          } else if ((!(balanceTimer.hasElapsed(0.1)) && drivetrain.getNavx().getPitch() < Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES)){
              balanceTimer.start();
          //if the pitch changes to greater than the tolerance, stop and reset the balance timer
          } else if (drivetrain.getNavx().getPitch() > Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES){
              balanceTimer.stop();
              balanceTimer.reset();
        }
        }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
