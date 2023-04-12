// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, and drive to score second gamepiece.")
@DashboardAutoRequirements(requirements = { Grid.class, Node.class, ChargeStation.class })
public class ScoreTwoUpperPickupAuto extends ScoreTwoUpperAuto {
    public ScoreTwoUpperPickupAuto(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }
    
    @Override
    public void init() {
        super.init();

        final Translation2d nearChargeStationMidPoint = createTranslation2dInches(32, -12);
        final Pose2d nearCableProtectorPose = createPose2dInches(70, -12, 0);
        final Pose2d farCableProtectorPose = createPose2dInches(106, -12, 0);
        final Translation2d farChargeStationMidPoint = createTranslation2dInches(154, -6);
        final Pose2d startPickUpPose = createPose2dInches(168, -36, 0);
        final Pose2d pickUpPose = createPose2dInches(202, -48, 0);

        final AutoTrajectoryConfig backupTrajectoryConfig = new AutoTrajectoryConfig(3.5, 3, 1, 4, 5, 0, 1);
        final AutoTrajectoryConfig cableProtectorTrajectoryConfig = new AutoTrajectoryConfig(1, 1, 1, 3, 2, 1, 1);
        final AutoTrajectoryConfig pickupLineUpTrajectoryConfig = new AutoTrajectoryConfig(4, 4, 2.5, 3, 2, 1, 3);
        final AutoTrajectoryConfig pickupTrajectoryConfig = new AutoTrajectoryConfig(3, 3, 1, 4, 2, 2.5, 0);
        
        // Drive back slightly and retract to avoid rotation collisions with the grid.
        SwerveControllerCommand backupPath = createSwerveCommand(
            backupTrajectoryConfig, 
            getLastEndingPose(),
            List.of(nearChargeStationMidPoint), 
            nearCableProtectorPose,
            createRotation(0)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            backupPath,
            new IntakeStopCommand(autoRequirements.getIntake()),
            new ElevatorPositionCommand(40000, autoRequirements.getElevator()).beforeStarting(new WaitCommand(0.25))
        );
        addCommands(retractGroup);

        SwerveControllerCommand cableProtectorPath = createSwerveCommand(
            cableProtectorTrajectoryConfig, 
            getLastEndingPose(), 
            farCableProtectorPose,
            createRotation(0)
        );

        addCommands(cableProtectorPath);

        // Slow down over cable protector to avoid odometry drift.        
        SwerveControllerCommand pickupLineUpCommand = createSwerveCommand(
            pickupLineUpTrajectoryConfig, 
            getLastEndingPose(),
            List.of(farChargeStationMidPoint),
            startPickUpPose,
            createRotation(-45)
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

        addCommands(new ArmInCommand(autoRequirements.getArm()));
    }
}
