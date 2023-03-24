// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends DriveCommand {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier gyroControlSupplier;
    private final BooleanSupplier fieldCentricSupplier;
    
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotationLimiter;

    private final ProfiledPIDController gyroRotationController;

    /**
     * Creates a new defaultDriveCommand.
     * 
     * @param xSupplier supplier for forward velocity.
     * @param ySupplier supplier for sideways velocity.
     * @param rotationSupplier supplier for angular velocity.
     */
    public DefaultDriveCommand(
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier rotationSupplier,
        BooleanSupplier gyroControlSupplier,
        BooleanSupplier fieldCentricSupplier,
        DrivetrainSubsystem drivetrain
    ) {
        super(drivetrain);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.gyroControlSupplier = gyroControlSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;

        this.gyroRotationController = new ProfiledPIDController(
            0.004, 0.001, 0,
            new TrapezoidProfile.Constraints(0.25, 0.25)
        );
        gyroRotationController.setTolerance(0.1);

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);
        rotationLimiter = new SlewRateLimiter(2);
    }

    @Override
    protected void drive() {
        double rotation = slewAxis(rotationLimiter, deadBand(-rotationSupplier.getAsDouble() * .75));
        if (gyroControlSupplier.getAsBoolean()) {
            double robotAngleDegrees = drivetrain.getRotation().getDegrees();
            rotation = gyroRotationController.calculate(robotAngleDegrees, Math.copySign(180, robotAngleDegrees));
            System.out.println(rotation);
        }

        drivetrain.drive(
            slewAxis(xLimiter, deadBand(-xSupplier.getAsDouble())), 
            slewAxis(yLimiter, deadBand(-ySupplier.getAsDouble())),
            rotation,
            fieldCentricSupplier.getAsBoolean()
        );
    }
}
