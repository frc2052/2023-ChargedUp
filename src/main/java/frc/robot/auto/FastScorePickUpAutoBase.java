// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoConfiguration;
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

public class FastScorePickUpAutoBase extends AutoBase {
    public FastScorePickUpAutoBase(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }

    @Override
    public void init() {
        final double startingYOffset = getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), 
            Node.MIDDLE_CUBE
        );

        final Pose2d initialPose = createPose2dInches(12.25, startingYOffset, 0);
        final Pose2d chargeStationPose = createPose2dInches(70, -14, 0);
        final Pose2d startPickUpPose = createPose2dInches(160, -14, 0);
        final Pose2d pickUpPose = createPose2dInches(202, -16, 0);

        final AutoTrajectoryConfig backupTrajectoryConfig = new AutoTrajectoryConfig(3.5, 3, 1, 4, 5, 0, 3);
        final AutoTrajectoryConfig pickupLineUpTrajectoryConfig = new AutoTrajectoryConfig(4, 4, 2.5, 4, 2, 3, 2);
        final AutoTrajectoryConfig pickupTrajectoryConfig = new AutoTrajectoryConfig(3, 3, 1, 4, 2, 2, 0);

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
            chargeStationPose,
            createRotation(-5)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            backupPath, 
            new ElevatorPositionCommand(40000, autoRequirements.getElevator())
        );
        addCommands(retractGroup);

        // Slow down over cable protector to avoid odometry drift.        
        SwerveControllerCommand pickupLineUpCommand = createSwerveCommand(
            pickupLineUpTrajectoryConfig, 
            getLastEndingPose(),
            startPickUpPose,
            createRotation(0)
        );
        ParallelDeadlineGroup pickupLineUpGroup = new ParallelDeadlineGroup(
            pickupLineUpCommand,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator()).beforeStarting(
                new WaitCommand(0.125)
            )                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
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
