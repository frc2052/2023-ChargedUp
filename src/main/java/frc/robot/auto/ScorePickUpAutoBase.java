// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;
import frc.robot.subsystems.IntakeSubsystem;

public class ScorePickUpAutoBase extends AutoBase {
    private final ForwardPixySubsystem forwardPixy;

    public ScorePickUpAutoBase(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm,
        ForwardPixySubsystem forwardPixy
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);

        this.forwardPixy = forwardPixy;
    }

    @Override
    public void init() {
        double startingYOffset = getStartingYOffsetInches(
            autoConfiguration.getStartingGrid(), 
            autoConfiguration.getStartingNode()
        );

        Pose2d initialPose = createPose2dInches(0, startingYOffset, 0);
        Pose2d bumpBackupPose2d = createPose2dInches(18, -6, 0);
        Translation2d chargeStationMidpoint = createTranslation2dInches(48, -4);
        Pose2d startPickUpPose = createPose2dInches(118, -4, 0);
        final double pickUpXMeters = Units.inchesToMeters(202);

        AutoTrajectoryConfig retractTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 1.5, 0, 1);
        AutoTrajectoryConfig backupTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 1, 1);

        addCommands(new ResetOdometryCommand(drivetrain, initialPose));

        // Initial score command.
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }
        addCommands(new ScoreCommand(
            () -> ScoreMode.CONE, 
            () -> autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0.125 : 0.25,
            intake
        ));

        // Drive back slightly to avoid rotation collisions with the grid.
        SwerveControllerCommand retractPath = createSwerveTrajectoryCommand(
            retractTrajectoryConfig, 
            initialPose, 
            bumpBackupPose2d,
            createRotation(90)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            retractPath, 
            new CompleteScoreCommand(elevator, intake, arm)
        );
        addCommands(retractGroup);

        // Slow down over cable protector to avoid odometry drift.
        SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
            backupTrajectoryConfig, 
            getLastEndingPose(),
            List.of(chargeStationMidpoint),
            startPickUpPose,
            createRotation(0)
        );
        ParallelDeadlineGroup backUpGroup = new ParallelDeadlineGroup(
            backupPath,
            new ElevatorPositionCommand(ElevatorPosition.FLOOR_CUBE, elevator)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        );
        addCommands(backUpGroup);

        // Drive to approach and pick up the cone.
        GamePieceAlignmentCommand pickupCommand = new GamePieceAlignmentCommand(
            () -> 2,
            () -> pickUpXMeters,
            forwardPixy, 
            drivetrain
        );
        ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
            pickupCommand,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_PICKUP, elevator),
            new ArmOutCommand(arm),
            new IntakeInCommand(intake)
        );
        addCommands(pickUpGroup);

        addCommands(new InstantCommand(() -> setLastEndingPose(drivetrain.getPosition())));

        // Retract arm for the start of the next path.
        addCommands(new ArmInCommand(arm));

        // Pose2d initialPose = createPose2dInches(0, getLeftStartingYOffsetInches(startGrid, startNode) * flip, 0);
        // Translation2d chargeStationMidpoint = createTranslation2dInches(18, 6 * flip);
        // Pose2d startPickUpPose = createPose2dInches(64, 4 * flip, 0);
        // Pose2d pickUpPose = createPose2dInches(194, 12 * flip, 0);

        // drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));

        // // Score first time
        // if (startNode == Node.MIDDLE_CUBE) {
        //     addCommands(new MidScoreCommand(elevator, arm));
        // } else {
        //     addCommands(new TopScoreCommand(elevator, arm));
        // }

        // SwerveControllerCommand backupPath = createSwerveTrajectoryCommand(
        //     AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(2), 
        //     initialPose,
        //     List.of(chargeStationMidpoint),
        //     startPickUpPose,
        //     createRotation(0)
        // );

        // ParallelCommandGroup retractGroup = new ParallelCommandGroup(
        //     new ScoreCommand(intake, arm, elevator, startNode == Node.MIDDLE_CUBE).withTimeout(
        //         startNode == Node.MIDDLE_CUBE ? 0 : 0.5
        //     ),
        //     backupPath.beforeStarting(new WaitCommand(0.5))
        // );
        // addCommands(retractGroup);

        // // Drive to approach cone
        // SwerveControllerCommand pickupPath = createSwerveTrajectoryCommand(
        //     AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig.withStartVelocity(2), 
        //     getLastEndingPose(), 
        //     pickUpPose,
        //     createRotation(0)
        // );

        // ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
        //     pickupPath,
        //     new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator),
        //     new ArmOutCommand(arm),
        //     new IntakeInCommand(intake)
        // );
        // addCommands(pickUpGroup);
        // addCommands(new ArmInCommand(arm));
    }
}
