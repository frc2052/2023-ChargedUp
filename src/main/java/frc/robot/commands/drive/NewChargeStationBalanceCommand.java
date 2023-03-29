// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class NewChargeStationBalanceCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;

    private boolean balanced;
    private boolean flipped = false;
    private boolean isDropping;

    private Timer dropTimer;

    private double previousPitch;
    private double currentPitch;

    /** Creates a new NewChargeStationAutoBalance. */
    public NewChargeStationBalanceCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        dropTimer = new Timer();
        
        addRequirements(drivetrain);
    }

    public void reset() {
        previousPitch = 0;
        currentPitch = 0;
        dropTimer.stop();
        dropTimer.reset();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        balanced = false;
        previousPitch = 0;
        currentPitch = 0;
    }

    @Override
    public void execute(){
        previousPitch = currentPitch;
        currentPitch = drivetrain.getNavx().getPitch();
        isDropping = Math.abs(previousPitch) - Math.abs(currentPitch) > 0.1 && Math.abs(currentPitch) < 10;
        balanced = Math.abs(currentPitch) < Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES;

        //check if the match is almost done and x wheels if so
        if (DriverStation.getMatchTime() <= 0.25) {
            drivetrain.xWheels();
            return;
        }

        if (!balanced && !flipped && !isDropping) {

            drivetrain.drive(
                Math.copySign(0.08, (double) -(drivetrain.getNavx().getPitch())),
                Math.copySign(0.065, (double) -(drivetrain.getNavx().getPitch())),
                0, 
                false
            );

            if (!(Math.copySign(1, currentPitch) == Math.copySign(1, previousPitch))){
                flipped = !flipped;
            }

            if (dropTimer.hasElapsed(1) && !balanced){
                dropTimer.reset();
            } else if (dropTimer.hasElapsed(0.5) && balanced) {
                drivetrain.xWheels();
                return;
            } else if (dropTimer.get() == 0 && isDropping) {
                dropTimer.start();
            }
        } else if (!balanced && flipped && !isDropping) {

            drivetrain.drive(
                Math.copySign(0.06, (double) -(drivetrain.getNavx().getPitch())),
                Math.copySign(0.04, (double) -(drivetrain.getNavx().getPitch())),
                0, 
                false
            );

            if (!(Math.copySign(1, currentPitch) == Math.copySign(1, previousPitch))){
                flipped = !flipped;
            }

            if (dropTimer.hasElapsed(0.5) && balanced) {
                drivetrain.xWheels();
                return;
            } else if (dropTimer.get() == 0 && isDropping) {
                dropTimer.start();
            } else if (dropTimer.hasElapsed(1) && !balanced){
                dropTimer.reset();
            }
        } else {
            drivetrain.xWheels();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
