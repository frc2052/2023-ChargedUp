// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.ElevatorSubsystem;
import frc.robot.subsystems.drive.ElevatorSubsystem.ElevatorPosition;

public class ElevatorPositionCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    
    private final ElevatorPosition position;
  
    /** Creates a new ElevatorPositionCommand. */
    public ElevatorPositionCommand(ElevatorPosition position, ElevatorSubsystem elevator) {
        this.elevator = elevator;

        this.position = position;

        addRequirements(this.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setPosition(position);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }
}