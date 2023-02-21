// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class ScoreCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;

    public ScoreCommand(IntakeSubsystem intake, ArmSubsystem arm, ElevatorSubsystem elevator) {
        this.intake = intake;
        this.arm = arm;
        this.elevator = elevator;

        addRequirements(this.intake, this.arm, this.elevator);
    }

    @Override
    public void initialize() {
        intake.intakeOut();
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
        return false;
    }
}
