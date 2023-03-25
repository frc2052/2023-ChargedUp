// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drive.DumbHorizontalAlignmentCommand;
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

/** Add your docs here. */
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

        Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(108, -2);
        Translation2d nearChargeStationInterpolationPoint = createTranslation2dInches(18, -6);

        Pose2d lineUpPose = createPose2dInches(
            10, 
            getStartingYOffsetInches(autoConfiguration.getStartingGrid(), Node.LEFT_CONE), 
            180
        );
        Pose2d scorePose = createPose2dInches(5, 0, 0);

        // Driving back to grid
        SwerveControllerCommand driveBackPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(2),
            getLastEndingPose(), 
            List.of(farChargeStationInterpolationPoint), 
            lineUpPose, 
            createRotation(180)
        );
        addCommands(driveBackPath);

        addCommands(new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator));

        addCommands(
            new DumbHorizontalAlignmentCommand(
                drivetrain,
                vision,
                pixy, 
                () -> 0, 
                () -> 0
            ).withTimeout(4)
        );

        addCommands(new TopScoreCommand(elevator, arm));
        addCommands(new ScoreCommand(intake, arm, elevator).withTimeout(0.5));
    }
}
