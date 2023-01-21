// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final Supplier<Rotation2d> rotationSupplier;
    
    private final DrivetrainSubsystem drivetrain;

    /** Creates a new defaultDriveCommand. */
    public DefaultDriveCommand(
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        Supplier<Rotation2d> rotationSupplier,
        DrivetrainSubsystem drivetrain
    ) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;

        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean fieldCentric = false;
        
        if (fieldCentric) {
            // Field centric driving
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                slewAxis(deadBand(xSupplier.getAsDouble())), 
                slewAxis(deadBand(ySupplier.getAsDouble())), 
                slewAxis(deadBand(rotationSupplier.get().getRadians())), 
                drivetrain.getRotation()
            ));
        } else {
            // Driver centric driving
            drivetrain.drive(new ChassisSpeeds(
                slewAxis(deadBand(xSupplier.getAsDouble())), 
                slewAxis(deadBand(ySupplier.getAsDouble())), 
                slewAxis(deadBand(rotationSupplier.get().getRadians())) 
            ));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    private double slewAxis(double value) {
        return new SlewRateLimiter(0.2).calculate(Math.copySign(Math.pow(value, 2), value));
    }

    private double deadBand(double value) {
        if (Math.abs(value) <= 0.05) {
            return 0;
        }
        return value;
    }
}
