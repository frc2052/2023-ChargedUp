// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class MidScoreCommand extends ParallelCommandGroup {
    /** Creates a new TopScoreCommand. */
    public MidScoreCommand(ElevatorSubsystem elevator, ArmSubsystem arm) {
        addCommands(
            new ElevatorPositionCommand(ElevatorPosition.MID_SCORE, elevator),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ArmOutCommand(arm)
            )
        );
    }
}
