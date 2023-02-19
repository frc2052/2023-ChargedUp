// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.io.Dashboard.Node;
import frc.robot.io.Dashboard.Row;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class AutomaticScoringCommand extends SequentialCommandGroup {
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final DrivetrainSubsystem drivetrain;

    /** Creates a new AutomaticScoringCommand. */
    public AutomaticScoringCommand(
        Node node,
        Row row,
        ElevatorPosition elevatorPosition,
        ElevatorSubsystem elevator,
        IntakeSubsystem intake,
        ArmSubsystem arm,
        DrivetrainSubsystem drivetrain
    ) {

        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;
        this.drivetrain = drivetrain;

        switch (row) {
            case HYBRID:
                

                break;

            case MIDDLE:

                break;
        
            case HIGH:

                break;
        }

        addCommands(
            


        );

        addRequirements(
            this.elevator,
            this.intake,
            this.arm,
            this.drivetrain
        );
    }
}
