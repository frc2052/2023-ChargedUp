// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakeOutCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.intakeOut();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
