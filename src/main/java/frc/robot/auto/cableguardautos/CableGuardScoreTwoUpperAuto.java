// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.cableguardautos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.drive.DumbHorizontalAlignmentCommand;
import frc.robot.commands.drive.GyroAlignmentCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, and drive to score second gamepiece.")
@DashboardAutoRequirements(requirements = { Grid.class, Node.class, ChargeStation.class })
public class CableGuardScoreTwoUpperAuto extends CableGuardFastScorePickUpAutoBase {
    public CableGuardScoreTwoUpperAuto(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }
    
    @Override
    public void init() {
        super.init();

        final Pose2d farCableProtectorPose = createPose2dInches(106, -12, 180);
        final Pose2d nearCableProtectorPose = createPose2dInches(70, -12, 180);
        final Translation2d chargeStationMidPoint = createTranslation2dInches(32, -12);
        final Pose2d lineUpPose = createPose2dInches(
            autoConfiguration.getStartingGrid() == Grid.RIGHT_GRID ? 16 : 21.999937, 
            autoConfiguration.getStartingGrid() == Grid.RIGHT_GRID ? -32 : -40,
            autoConfiguration.getStartingGrid() == Grid.RIGHT_GRID ? 225 : 135
        );

        final AutoTrajectoryConfig driveBackTrajectoryConfig = new AutoTrajectoryConfig(4, 4, 2.5, 4, 4.5, 0, 1);
        final AutoTrajectoryConfig cableProtectorTrajectoryConfig = new AutoTrajectoryConfig(1, 1, 1, 3, 2, 1, 1.5);
        final AutoTrajectoryConfig lineUpTrajectoryConfig = new AutoTrajectoryConfig(2, 1, 1, 4, 2, 1.5, 0);

        // Driving back to the grid.
        SwerveControllerCommand driveBackPath = createSwerveCommand(
            driveBackTrajectoryConfig,
            getLastEndingPose(),
            farCableProtectorPose, 
            createRotation(-175)
        );
        ParallelDeadlineGroup driveBackGroup = new ParallelDeadlineGroup(
            driveBackPath,
            new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, autoRequirements.getElevator())
        );
        addCommands(driveBackGroup);

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
        ParallelCommandGroup lineUpGroup = new ParallelCommandGroup(
            lineUpPath,
            new InstantCommand(autoRequirements.getIntakePixy()::updateConePosition, autoRequirements.getIntakePixy()).andThen(
                new TopScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm())
            )
        );

        addCommands(lineUpGroup);

        // Line up and bring up elevator to score.
        addCommands(new GyroAlignmentCommand(() -> Rotation2d.fromDegrees(180), () -> true, autoRequirements.getDrivetrain()));
        
        addCommands(
            new DumbHorizontalAlignmentCommand(
                () -> -0.2, 
                () -> 0.0, 
                autoRequirements.getDrivetrain(), 
                autoRequirements.getVision(), 
                autoRequirements.getIntakePixy()
            ).withTimeout(0.5)
        );
        addCommands(
            new DumbHorizontalAlignmentCommand(
                () -> 0.25, 
                () -> 0.0, 
                autoRequirements.getDrivetrain(), 
                autoRequirements.getVision(), 
                autoRequirements.getIntakePixy()
            ).withTimeout(0.75)
        );
        setLastEndingPose(createPose2dInches(0, -35, 0));

        // Second score and retract.
        addCommands(new InstantCommand(() -> autoRequirements.getIntake().setScoreMode(ScoreMode.CONE)));
        addCommands(new ScoreCommand(() -> 0.25, autoRequirements.getIntake()));
    }
}
