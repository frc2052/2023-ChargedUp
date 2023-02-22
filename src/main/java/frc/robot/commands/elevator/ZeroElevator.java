// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends CommandBase {
    private final ElevatorSubsystem elevator;

    /** Creates a new ZeroElevator. */
    public ZeroElevator(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.manualDown();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elevator.elevatorZeroed();
    }
}
