// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public abstract class DriveCommand extends CommandBase {
    protected final DrivetrainSubsystem drivetrain;

    /** Creates a new DriveCommand. */
    public DriveCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive();
    }

    protected abstract void drive();

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
