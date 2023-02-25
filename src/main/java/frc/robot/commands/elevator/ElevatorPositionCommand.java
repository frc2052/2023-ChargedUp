// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class ElevatorPositionCommand extends CommandBase {
    private final ElevatorPosition position;

    private final ElevatorSubsystem elevator;
  
    /** Creates a new ElevatorPositionCommand. */
    public ElevatorPositionCommand(
        ElevatorPosition position, 
        ElevatorSubsystem elevator
    ) {
        this.position = position;

        this.elevator = elevator;

        addRequirements(this.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setPosition(position);
    }

    @Override
    public void end(boolean interupted) {
        elevator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }
}
