// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualUpCommand extends CommandBase {
    private final ElevatorSubsystem elevator;

    public ElevatorManualUpCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        elevator.manualUp();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
