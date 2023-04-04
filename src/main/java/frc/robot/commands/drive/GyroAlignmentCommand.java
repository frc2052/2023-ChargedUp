// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class GyroAlignmentCommand extends DriveCommand {
    private final PIDController rotationController;

    public GyroAlignmentCommand(Supplier<Rotation2d> targetRotation, DrivetrainSubsystem drivetrain) {
        super(() -> 0, () -> 0, () -> 0, () -> false, drivetrain);

        rotationController = new PIDController(2, 0, 0);
        rotationController.enableContinuousInput(0, 360);
        rotationController.setSetpoint(targetRotation.get().getDegrees());
        rotationController.setTolerance(1);
    }

    @Override
    protected double getRotation() {
        // Forcing angle to be between [0, 360], PIDController thinks -180 isn't at setpoint of 180
        double gyroDegrees = (drivetrain.getRotation().getDegrees() + 360) % 360;

        return rotationController.calculate(gyroDegrees) / 360;
    }

    @Override
    public boolean isFinished() {
        return rotationController.atSetpoint();
        //return Math.abs(gyroDegrees - 180) <= 2;
    }
}
