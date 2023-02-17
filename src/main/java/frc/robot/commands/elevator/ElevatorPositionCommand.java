// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class ElevatorPositionCommand extends CommandBase {
    private final ElevatorPosition position;

    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
  
    /** Creates a new ElevatorPositionCommand. */
    public ElevatorPositionCommand(
        ElevatorPosition position, 
        ElevatorSubsystem elevator, 
        ArmSubsystem arm
    ) {
        this.position = position;

        this.elevator = elevator;
        this.arm = arm;

        addRequirements(this.elevator, this.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        switch (position) {
            case TOP_SCORE:
            case MID_SCORE:
                arm.armOut();
                break;
        
            default:
                arm.armIn();
                break;
        }

        elevator.setPosition(position);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }
}
