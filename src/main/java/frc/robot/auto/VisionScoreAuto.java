// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.DumbHorizontalAlignmentCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

/** Add your docs here. */
public class VisionScoreAuto extends AutoBase {
    private final VisionSubsystem vision;
    private final PixySubsystem pixy;

    public VisionScoreAuto(
        AutoConfiguration autoConfiguration, 
        DrivetrainSubsystem drivetrain,
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm,
        VisionSubsystem vision,
        PixySubsystem pixy
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);

        this.vision = vision;
        this.pixy = pixy;
    }

    @Override
    public void init() {
        addCommands(new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator));

        addCommands(new InstantCommand(() -> { pixy.updateConePosition(); }));

        addCommands(
            new DumbHorizontalAlignmentCommand(
                drivetrain,
                vision,
                pixy, 
                () -> 0.25,
                () -> 0
            ).withTimeout(2)
        );
        
        addCommands(new TopScoreCommand(elevator, arm));
        addCommands(new ScoreCommand(intake, arm, elevator).withTimeout(0.5));
    }
}
