// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

public class IntakeOutCommand extends CommandBase {
    private final IntakeSubsystem intake;

    private final Supplier<ScoreMode> scoreMode;

    public IntakeOutCommand(Supplier<ScoreMode> scoreMode, IntakeSubsystem intake) {
        this.intake = intake;

        this.scoreMode = scoreMode;

        addRequirements(this.intake);
    }

    public IntakeOutCommand(IntakeSubsystem intake) {
        this(() -> ScoreMode.CONE, intake);
    }

    @Override
    public void execute() {
        intake.intakeOut(scoreMode.get());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
