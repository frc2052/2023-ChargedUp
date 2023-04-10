// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreCommand extends CommandBase {
    private final IntakeSubsystem intake;

    private final Timer scoreTimer;
    private final DoubleSupplier maxScoreTime;

    public ScoreCommand(IntakeSubsystem intake) {
        this(() -> 0, intake);
    }

    public ScoreCommand(DoubleSupplier maxScoreTime, IntakeSubsystem intake) {
        this.intake = intake;

        scoreTimer = new Timer();
        this.maxScoreTime = maxScoreTime;
        
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        scoreTimer.reset();
        scoreTimer.start();
    }

    @Override
    public void execute() {
        intake.intakeOut();
    }

    // Score command ends when interupted or timed out.
    @Override
    public boolean isFinished() {
        return scoreTimer.hasElapsed(maxScoreTime.getAsDouble());
    }
}
