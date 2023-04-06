// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

public class TimedScoreCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;

    private final ScoreMode scoreMode;
    private final double scoreTime;
    private final Timer scoreTimer;

    public TimedScoreCommand(ScoreMode scoreMode, double scoreTime, IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator) {
        this.intake = intake;
        this.arm = arm;
        this.elevator = elevator;

        this.scoreMode = scoreMode;
        this.scoreTime = scoreTime;
        scoreTimer = new Timer();

        addRequirements(this.intake, this.arm, this.elevator);
    }

    @Override
    public void initialize() {
        scoreTimer.reset();
        scoreTimer.start();
    }

    @Override
    public void execute() {
        if (elevator.atPosition()) {
            intake.intakeOut(scoreMode);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        arm.armIn();
        elevator.setPosition(ElevatorPosition.BABY_BIRD);
    }

    // Score command ends when interupted or timed out.
    @Override
    public boolean isFinished() {
        return scoreTimer.hasElapsed(scoreTime);
    }
}
