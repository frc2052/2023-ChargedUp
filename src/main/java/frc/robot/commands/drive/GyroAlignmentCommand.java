// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class GyroAlignmentCommand extends DriveCommand {
    private final PIDController rotationController;
    
    private double gyroDegrees;

    public GyroAlignmentCommand(DrivetrainSubsystem drivetrain) {
        super(drivetrain);

        rotationController = new PIDController(1, 0, 0);
        rotationController.enableContinuousInput(0, 360);
    }
    
    @Override
    protected void drive() {
        // Forcing angle to be between [0, 360], PIDController thinks -180 isn't at setpoint of 180
        gyroDegrees = (drivetrain.getRotation().getDegrees() + 360) % 360;

        drivetrain.drive(
            0,
            0,
            rotationController.calculate(gyroDegrees, 180) / 180,
            false
        );
    }



    @Override
    public boolean isFinished() {
        return Math.abs(gyroDegrees - 180) <= 1;
    }
}
