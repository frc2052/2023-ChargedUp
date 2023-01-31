// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorUpCommand extends CommandBase {
    private final ElevatorSubsystem elevator;

    /** Creates a new StopElevatorCommand. */
    public ManualElevatorUpCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;

        addRequirements(this.elevator);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.manualUp();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}


