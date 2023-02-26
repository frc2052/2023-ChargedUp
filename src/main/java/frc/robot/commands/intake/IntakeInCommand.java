// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends CommandBase {
    private final BooleanSupplier armState;

    private final IntakeSubsystem intake;

    public IntakeInCommand(IntakeSubsystem intake) {
        this(() -> true, intake);
    }

    public IntakeInCommand(BooleanSupplier armState, IntakeSubsystem intake) {
        this.armState = armState;
       
        this.intake = intake;

        addRequirements(this.intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        if (armState.getAsBoolean()) {
            intake.intakeIn();
        } else {
            intake.slowIntakeIn();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
