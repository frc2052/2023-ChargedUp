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
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.FastScorePickUpAutoBase;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.drive.DumbHorizontalAlignmentCommand;
import frc.robot.commands.drive.GyroAlignmentCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, and drive to score second gamepiece.")
@DashboardAutoRequirements(requirements = { Grid.class, Node.class, ChargeStation.class })
public class ScoreTwoUpperAuto extends FastScorePickUpAutoBase {
    public ScoreTwoUpperAuto(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }
    
    @Override
    public void init() {
        super.init();
        
        final double yOffsetInches = getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(),
            autoConfiguration.getStartingGrid() == Grid.LEFT_GRID ? Node.RIGHT_CONE : Node.LEFT_CONE
        ) * (autoConfiguration.getStartingGrid() == Grid.LEFT_GRID ? 1 : -1);
        
        final Pose2d farCableProtectorPose = createPose2dInches(106, 2, 180);
        final Pose2d nearCableProtectorPose = createPose2dInches(70, 2, 180);
        final Translation2d chargeStationMidPoint = createTranslation2dInches(18, yOffsetInches / 2);
        final Pose2d lineUpPose = createPose2dInches(12, yOffsetInches, 225);

        final AutoTrajectoryConfig driveBackTrajectoryConfig = new AutoTrajectoryConfig(4, 3.5, 2.5, 4, 2, 0, 1);
        final AutoTrajectoryConfig cableProtectorTrajectoryConfig = new AutoTrajectoryConfig(1, 1, 1, 3, 2, 1, 1);
        final AutoTrajectoryConfig lineUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 1, 0);

        // Driving back to the grid.
        SwerveControllerCommand driveBackPath = createSwerveCommand(
            driveBackTrajectoryConfig,
            getLastEndingPose(),
            farCableProtectorPose, 
            createRotation(-175)
        );
        addCommands(driveBackPath);

        SwerveControllerCommand cableProtectorPath = createSwerveCommand(
            cableProtectorTrajectoryConfig,
            getLastEndingPose(),
            nearCableProtectorPose, 
            createRotation(180)
        );
        addCommands(cableProtectorPath);

        SwerveControllerCommand lineUpPath = createSwerveCommand(
            lineUpTrajectoryConfig,
            getLastEndingPose(),
            List.of(chargeStationMidPoint),
            lineUpPose, 
            createRotation(180)
        );
        addCommands(lineUpPath);
        
        // Line up and bring up elevator to score.
        addCommands(new GyroAlignmentCommand(() -> Rotation2d.fromDegrees(180), () -> true, autoRequirements.getDrivetrain()));
        
        ParallelCommandGroup scoreGroup = new ParallelCommandGroup(
            new DumbHorizontalAlignmentCommand(
                () -> 0.35, 
                () -> 0.0, 
                autoRequirements.getDrivetrain(), 
                autoRequirements.getVision(), 
                autoRequirements.getIntakePixy()
            ).withTimeout(1),
            new TopScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm())
        );
        addCommands(scoreGroup);

        // Second score and retract.
        addCommands(new InstantCommand(() -> autoRequirements.getIntake().setScoreMode(ScoreMode.CONE)));
        addCommands(new ScoreCommand(() -> 0.25, autoRequirements.getIntake()));
        addCommands(new CompleteScoreCommand(autoRequirements.getElevator(), autoRequirements.getIntake(), autoRequirements.getArm()));
    }
}
