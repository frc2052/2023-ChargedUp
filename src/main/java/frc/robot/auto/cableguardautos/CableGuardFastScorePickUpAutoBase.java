// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.cableguardautos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class CableGuardFastScorePickUpAutoBase extends AutoBase {
    public CableGuardFastScorePickUpAutoBase(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }

    @Override
    public void init() {
        final double startingYOffset = getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), 
            Node.MIDDLE_CUBE
        );

        final Pose2d initialPose = createPose2dInches(12.25, startingYOffset, 0);
        final Pose2d nearCableProtectorPose = createPose2dInches(70, -2, 0);
        final Pose2d farCableProtectorPose = createPose2dInches(106, -2, 0);
        final Pose2d startPickUpPose = createPose2dInches(150, -12, 0);
        final Pose2d pickUpPose = createPose2dInches(208, -16, autoConfiguration.getStartingGrid() == Grid.RIGHT_GRID ? 165 : 195);

        final AutoTrajectoryConfig backupTrajectoryConfig = new AutoTrajectoryConfig(3.5, 3, 1, 4, 5, 0, 1);
        final AutoTrajectoryConfig cableProtectorTrajectoryConfig = new AutoTrajectoryConfig(1, 1, 1, 3, 2, 1, 1);
        final AutoTrajectoryConfig pickupLineUpTrajectoryConfig = new AutoTrajectoryConfig(4, 4, 2.5, 4, 2, 1, 3);
        final AutoTrajectoryConfig pickupTrajectoryConfig = new AutoTrajectoryConfig(3, 3, 1, 4, 2, 2.5, 0);

        addCommands(new ResetOdometryCommand(autoRequirements.getDrivetrain(), initialPose));
        
        // Initial elevator score command.
        addCommands(new ParallelCommandGroup(
            new ElevatorPositionCommand(87500, autoRequirements.getElevator()),
            new ArmOutCommand(autoRequirements.getArm()).beforeStarting(new WaitCommand(0.2))
        ));
        
        // Drive back slightly and retract to avoid rotation collisions with the grid.
        SwerveControllerCommand backupPath = createSwerveCommand(
            backupTrajectoryConfig, 
            initialPose, 
            nearCableProtectorPose,
            createRotation(-5)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            backupPath, 
            new ElevatorPositionCommand(40000, autoRequirements.getElevator())
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
            //).andThen(new ResetOdometryCommand(autoRequirements.getDrivetrain(), new Pose2d(pickUpPose.getX(), pickUpPose.getY(), 0)));
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
