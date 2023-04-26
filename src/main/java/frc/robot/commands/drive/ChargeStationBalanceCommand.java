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

    private boolean balanced;
    private boolean flipped;
    private boolean isDropping;

    private Timer slowdownTimer;
    private final Timer balanceTimer;

    private double previousPitch;
    private double currentPitch;

    /** Creates a new NewChargeStationAutoBalance. */
    public ChargeStationBalanceCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        slowdownTimer = new Timer();
        balanceTimer = new Timer();
        
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        balanced = false;
        flipped = false;
        previousPitch = 0;
        currentPitch = 0;

        slowdownTimer.start();
        slowdownTimer.reset();

        balanceTimer.stop();
        balanceTimer.reset();
    }

    @Override
    public void execute() {
        previousPitch = currentPitch;
        currentPitch = drivetrain.getNavx().getPitch();

        double deltaPitch = Math.abs(previousPitch) - Math.abs(currentPitch);
        isDropping = deltaPitch > 0.1 && Math.abs(currentPitch) < 11;
        balanced = Math.abs(currentPitch) < Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES;

        //check if the match is almost done and x wheels if so
        if (DriverStation.isFMSAttached() && DriverStation.getMatchTime() <= 0.25) {
            drivetrain.xWheels();
            return;
        }
        
        System.out.println("Pitch: " + currentPitch);
        
        if (previousPitch != 0 && Math.copySign(1, currentPitch) != Math.copySign(1, previousPitch)) {
            System.out.println("Flipped!");
            flipped = true;
            //drivetrain.setBalanced(true);
        }

        if (Math.abs(currentPitch) > 4 && flipped) {
            drivetrain.setBalanced(true);
        }

        if (!balanced) {
            balanceTimer.stop();
            balanceTimer.reset();
            //drivetrain.setBalanced(false);

            if (isDropping || flipped) {
                //System.out.println("BALANCE: Driving slow! dropping: " + isDropping);

                drivetrain.drive(
                    Math.copySign(0.04, (double) -(drivetrain.getNavx().getPitch())),
                    Math.copySign(0.04, (double) -(drivetrain.getNavx().getPitch())),
                    0, 
                    false
                );
            } else {
                //System.out.println("BALANCE: Driving fast!");

                if (slowdownTimer.hasElapsed(1.5)) {
                    drivetrain.drive(
                        Math.copySign(0.1, (double) -(drivetrain.getNavx().getPitch())),
                        Math.copySign(0.0, (double) -(drivetrain.getNavx().getPitch())),
                        0, 
                        false
                    );
                } else {
                    drivetrain.drive(
                        Math.copySign(0.15, (double) -(drivetrain.getNavx().getPitch())),
                        Math.copySign(0.0, (double) -(drivetrain.getNavx().getPitch())),
                        0, 
                        false
                    );
                }
            }
        } else {
            System.out.println("BALANCE: Balanced!");

            drivetrain.setBalanced(true);

            // balanceTimer.start();

            // if (balanceTimer.hasElapsed(0.1)) {
            //     drivetrain.setBalanced(true);
            // }

            drivetrain.xWheels();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
