// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChargeStationBalanceCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private boolean holding;
    private Timer balanceTimer;

    private double previousPitch;
    private double currentPitch;

    /** Creates a new NewChargeStationAutoBalance. */
    public ChargeStationBalanceCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        balanceTimer = new Timer();
        
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

    @Override
    public void execute(){
        previousPitch = currentPitch;
        currentPitch = drivetrain.getNavx().getPitch();
        boolean isDropping = Math.abs(previousPitch) - Math.abs(currentPitch) > 0.1 && Math.abs(currentPitch) < 10;
        boolean isLevel = Math.abs(drivetrain.getNavx().getPitch()) < 3;

        if (DriverStation.getMatchTime() <= 0.25) {
            drivetrain.xWheels();
            return;
        }

        if (!isDropping && !holding && !isLevel) {
            drivetrain.drive(
                Math.copySign(0.08, (double) -(drivetrain.getNavx().getPitch())),
                Math.copySign(0.065, (double) -(drivetrain.getNavx().getPitch())),
                0, 
                false
            );
        } else {
            //if balance timer has gone for 2 seconds, x the wheels
            if (balanceTimer.hasElapsed(0.5)) {
                drivetrain.xWheels();
                holding = true;
            //if the balance timer has NOT started and the bot is dropping, start timer
            } else if (balanceTimer.get() == 0 && isDropping) {
                balanceTimer.start();
            } else if (!balanceTimer.hasElapsed(1)) {
                //do nothing still dropping
            //if the pitch changes to greater than the tolerance, stop and reset the balance timer
            } else if ((Math.abs(drivetrain.getNavx().getPitch())) > Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES) {
                balanceTimer.stop();
                balanceTimer.reset();
                holding = false;
                isDropping = false;
                isLevel = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
