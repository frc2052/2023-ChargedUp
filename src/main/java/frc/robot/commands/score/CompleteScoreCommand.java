// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class CompleteScoreCommand extends ParallelCommandGroup {
    public CompleteScoreCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, ArmSubsystem arm) {
        addCommands(
            new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator).beforeStarting(new WaitCommand(0.4)),
            new IntakeStopCommand(intake),
            new ArmInCommand(arm)
        );
    }
}