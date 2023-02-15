// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends CommandBase {
    private final IntakeSubsystem intake;

    /** Creates a new ReverseIntake. */
    public IntakeInCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(this.intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.intakeIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
