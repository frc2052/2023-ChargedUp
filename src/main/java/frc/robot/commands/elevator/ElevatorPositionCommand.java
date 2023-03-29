// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class ElevatorPositionCommand extends CommandBase {
    private final ElevatorSubsystem elevator;

    private final ElevatorPosition position;
  
    public ElevatorPositionCommand(
        ElevatorPosition position, 
        ElevatorSubsystem elevator
    ) {
        this.elevator = elevator;

        this.position = position;

        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(position);
    }

    @Override
    public void end(boolean interupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }
}
