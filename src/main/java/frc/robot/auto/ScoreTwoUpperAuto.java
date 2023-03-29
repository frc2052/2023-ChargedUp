// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drive.DumbHorizontalAlignmentCommand;
import frc.robot.commands.drive.GyroAlignmentCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, and drive to score second gamepiece.")
public class ScoreTwoUpperAuto extends ScorePickUpAutoBase {
    private final VisionSubsystem vision;
    private final PixySubsystem pixy;

    public ScoreTwoUpperAuto(
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
        super.init();
        
        Translation2d chargeStationInterpolationMipoint = createTranslation2dInches(60, -12);

        Pose2d lineUpPose = createPose2dInches(12, getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), Node.RIGHT_CONE
        ), 225);

        // Drive back to cable protector.
        SwerveControllerCommand driveBackPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withEndVelocity(1),
            getLastEndingPose(),
            super.cableProtectorPoint, 
            createRotation(180)
        );

        ParallelDeadlineGroup driveBackGroup = new ParallelDeadlineGroup(
            driveBackPath,
            new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator).andThen(new RunCommand(pixy::updateConePosition))
        );
        addCommands(driveBackGroup);

        // Roughly line up with the scoring node.
        SwerveControllerCommand lineUpPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withStartVelocity(1),
            getLastEndingPose(),
            List.of(chargeStationInterpolationMipoint),
            lineUpPose, 
            createRotation(180)
        );
        addCommands(lineUpPath);

        addCommands(new GyroAlignmentCommand(drivetrain));
        addCommands(new DumbHorizontalAlignmentCommand(() -> 0.25, () -> 0.0, drivetrain, vision, pixy).withTimeout(1));
        
        addCommands(new TopScoreCommand(elevator, arm));
        addCommands(new ScoreCommand(intake, arm, elevator).withTimeout(0.5));
    }
}
