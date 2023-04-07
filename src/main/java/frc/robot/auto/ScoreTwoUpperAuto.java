// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drive.DumbHorizontalAlignmentCommand;
import frc.robot.commands.drive.GyroAlignmentCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.auto.AutoFactory.Grid;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakePixySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, and drive to score second gamepiece.")
public class ScoreTwoUpperAuto extends ScorePickUpAutoBase {
    private final VisionSubsystem vision;
    private final IntakePixySubsystem pixy;

    public ScoreTwoUpperAuto(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm,
        VisionSubsystem vision,
        IntakePixySubsystem pixy,
        ForwardPixySubsystem forwardPixy
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm, forwardPixy);

        this.vision = vision;
        this.pixy = pixy;
    }
    
    @Override
    public void init() {
        super.init();
        
        Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(108, -6);
        Translation2d nearChargeStationInterpolationPoint = createTranslation2dInches(18, -6);
        
        Translation2d channelInterpolationMipoint = createTranslation2dInches(60, -12);

        System.out.println(getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(),
            Node.MIDDLE_CUBE
        ));

        Pose2d lineUpPose = createPose2dInches(
            12, 
            getStartingYOffsetInches(
                autoConfiguration.getStartingGrid(),
                autoConfiguration.getStartingGrid() == Grid.LEFT_GRID ? Node.RIGHT_CONE : Node.LEFT_CONE
            ) * (autoConfiguration.getStartingGrid() == Grid.LEFT_GRID ? 1 : -1), 
            225
        );
//        Pose2d scorePose = createPose2dInches(6, 0, 0);

        // Driving back to grid
        SwerveControllerCommand driveBackPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig,
            getLastEndingPose(),
            List.of(channelInterpolationMipoint),
            lineUpPose, 
            createRotation(180)
        );

        ParallelDeadlineGroup driveBackGroup = new ParallelDeadlineGroup(
            driveBackPath,
            new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator).andThen(new RunCommand(pixy::updateConePosition))
        );

        addCommands(driveBackGroup);
        
        addCommands(new GyroAlignmentCommand(() -> Rotation2d.fromDegrees(180), drivetrain));
        addCommands(new DumbHorizontalAlignmentCommand(() -> 0.35, () -> 0.0, drivetrain, vision, pixy).withTimeout(1));
        
        addCommands(new TopScoreCommand(elevator, arm));
        
        addCommands(new ScoreCommand(
            () -> ScoreMode.CONE, 
            () -> 0.5,
            intake
        ).andThen(new CompleteScoreCommand(elevator, intake, arm)));

        // Translation2d farchargeStationMidpoint = createTranslation2dInches(130, -4);
        // Translation2d chargeStationInterpolationMipoint = createTranslation2dInches(48, -4);

        // Pose2d lineUpPose = createPose2dInches(12, getStartingYOffsetInches(
        //     autoConfiguration.getStartingGrid(), Node.RIGHT_CONE
        // ), 225);

        // // Drive back to cable protector.
        // SwerveControllerCommand driveBackPath = createSwerveTrajectoryCommand(
        //     AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withEndVelocity(2.5),
        //     getLastEndingPose(),
        //     // super.cableProtectorPoint, 
        //     List.of(farchargeStationMidpoint, chargeStationInterpolationMipoint),
        //     lineUpPose, 
        //     createRotation(180)
        // );

        // // Roughly line up with the scoring node.
        // SwerveControllerCommand lineUpPath = createSwerveTrajectoryCommand(
        //     AutoTrajectoryConfig.defaultTrajectoryConfig.withStartVelocity(2.5),
        //     getLastEndingPose(),
        //     List.of(chargeStationInterpolationMipoint),
        //     lineUpPose, 
        //     createRotation(180)
        // );
        // // addCommands(lineUpPath);

        // ParallelDeadlineGroup driveBackGroup = new ParallelDeadlineGroup(
        //     lineUpPath,
        //     new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator).andThen(new RunCommand(pixy::updateConePosition))
        // );
        // addCommands(driveBackGroup);

        // addCommands(new GyroAlignmentCommand(drivetrain));
        // addCommands(new DumbHorizontalAlignmentCommand(() -> 0.25, () -> 0.0, drivetrain, vision, pixy).withTimeout(1));
        
        // addCommands(new TopScoreCommand(elevator, arm));
        // addCommands(new ScoreCommand(intake, arm, elevator).withTimeout(0.5));
    }
}
