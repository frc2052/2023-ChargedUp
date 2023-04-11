// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.FastScorePickUpAutoBase;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.DumbHorizontalAlignmentCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.drive.GyroAlignmentCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.io.Dashboard;
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

        final Pose2d farCableProtectorPose = createPose2dInches(106, -2, 180);
        final Pose2d nearCableProtectorPose = createPose2dInches(70, -2, 180);
        final Translation2d chargeStationMidPoint = createTranslation2dInches(32, -12);
        final Pose2d lineUpPose = createPose2dInches(16, -30, 225);

        final AutoTrajectoryConfig driveBackTrajectoryConfig = new AutoTrajectoryConfig(4, 4, 2.5, 4, 4.5, 0, 1);
        final AutoTrajectoryConfig cableProtectorTrajectoryConfig = new AutoTrajectoryConfig(1, 1, 1, 3, 2, 1, 1);
        final AutoTrajectoryConfig lineUpTrajectoryConfig = new AutoTrajectoryConfig(3, 1.75, 1, 4, 2, 1, 0);

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
                () -> 0.25, 
                () -> 0.0, 
                autoRequirements.getDrivetrain(), 
                autoRequirements.getVision(), 
                autoRequirements.getIntakePixy()
            ).withTimeout(1)
        );
        setLastEndingPose(createPose2dInches(0, -35, 0));

        // Second score and retract.
        addCommands(new InstantCommand(() -> autoRequirements.getIntake().setScoreMode(ScoreMode.CONE)));
        addCommands(new ScoreCommand(() -> 0.25, autoRequirements.getIntake()));

        final Pose2d startPickUpPose = createPose2dInches(160, -12, 0);
        final Translation2d chargeStationAvoid = createTranslation2dInches(180, -24);
        final Pose2d pickUpPose = createPose2dInches(202, -48, 165);

        final AutoTrajectoryConfig backupTrajectoryConfig = new AutoTrajectoryConfig(3.5, 3, 1, 4, 5, 0, 1);
        final AutoTrajectoryConfig pickupLineUpTrajectoryConfig = new AutoTrajectoryConfig(4, 4, 2.5, 4, 2, 1, 3);
        final AutoTrajectoryConfig pickupTrajectoryConfig = new AutoTrajectoryConfig(3, 3, 1, 4, 2, 2.5, 0);
        
        // Drive back slightly and retract to avoid rotation collisions with the grid.
        SwerveControllerCommand backupPath = createSwerveCommand(
            backupTrajectoryConfig, 
            getLastEndingPose(),
            List.of(chargeStationMidPoint), 
            nearCableProtectorPose,
            createRotation(0)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            backupPath,
            new IntakeStopCommand(autoRequirements.getIntake()),
            new ElevatorPositionCommand(40000, autoRequirements.getElevator()).beforeStarting(new WaitCommand(0.5))
        );
        addCommands(retractGroup);

        SwerveControllerCommand secondCableProtectorPath = createSwerveCommand(
            cableProtectorTrajectoryConfig, 
            getLastEndingPose(), 
            farCableProtectorPose,
            createRotation(0)
        );

        addCommands(secondCableProtectorPath);

        // Slow down over cable protector to avoid odometry drift.        
        SwerveControllerCommand pickupLineUpCommand = createSwerveCommand(
            pickupLineUpTrajectoryConfig, 
            getLastEndingPose(),
            List.of(chargeStationAvoid),
            startPickUpPose,
            createRotation(0)
        );
        ParallelDeadlineGroup pickupLineUpGroup = new ParallelDeadlineGroup(
            pickupLineUpCommand,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator())                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        );
        addCommands(pickupLineUpGroup);

        // Drive to approach and pick up the cone.
        Command pickupCommand = null;
        if (!Dashboard.getInstance().pixyCamBroken()) {
            pickupCommand = new GamePieceAlignmentCommand(
                () -> pickUpPose.getX(),
                autoRequirements.getDrivetrain(),
                autoRequirements.getForwardPixy(),
                autoRequirements.getIntake()
            );
            setLastEndingPose(pickUpPose);
        } else {
            pickupCommand = createSwerveCommand(
                pickupTrajectoryConfig, 
                getLastEndingPose(),
                pickUpPose,
                createRotation(0)
            );
        }
        ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
            pickupCommand,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator()),
            new IntakeInCommand(autoRequirements.getIntake())
        );
        addCommands(pickUpGroup);

        addCommands(new ElevatorPositionCommand(40000, autoRequirements.getElevator()));
    }
}
