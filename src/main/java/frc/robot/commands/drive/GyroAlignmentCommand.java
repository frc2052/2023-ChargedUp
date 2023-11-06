// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class GyroAlignmentCommand extends DriveCommand {
    private final PIDController rotationController;

    private final Supplier<Rotation2d> targetRotation;
    private final BooleanSupplier shouldFinish;

    public GyroAlignmentCommand(
        Supplier<Rotation2d> targetRotation, 
        BooleanSupplier shouldFinish,
        DrivetrainSubsystem drivetrain
    ) {
        this(() -> 0, () -> 0, targetRotation, shouldFinish, drivetrain);
    }

    public GyroAlignmentCommand(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        Supplier<Rotation2d> targetRotation, 
        BooleanSupplier shouldFinish,
        DrivetrainSubsystem drivetrain
    ) {
        super(xSupplier, ySupplier, () -> 0, Dashboard.getInstance()::isFieldCentric, drivetrain);

        rotationController = new PIDController(2.5, 0, 0.1);
        rotationController.enableContinuousInput(0, 360);
        rotationController.setTolerance(2);

        this.targetRotation = targetRotation;

        this.shouldFinish = shouldFinish;
    }

    @Override
    public void initialize() {
        // Normalize the angle value between [0, 360]
        rotationController.setSetpoint((targetRotation.get().getDegrees() + 360) % 360);
    }

    @Override
    protected double getRotation() {
        // Forcing angle to be between [0, 360], PIDController thinks -180 isn't at setpoint of 180
        double gyroDegrees = (RobotState.getInstance().getRotation2d().getDegrees() + 360) % 360;

        // Calculate PID value along with feedforward constant to assist with minor adjustments.
        double rotationValue = rotationController.calculate(gyroDegrees) / 360;
        if (!rotationController.atSetpoint()) {
            return rotationValue + Math.copySign(0.025, rotationValue);
        } else {
            return 0;
        }
    }

    @Override
    public boolean isFinished() {
        return rotationController.atSetpoint() && shouldFinish.getAsBoolean();
    }
}
