// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.AutoFactory.GamePiece;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;
import frc.robot.subsystems.IntakeSubsystem;

@AutoDescription(description = "Score gamepiece, drive over charge station to pick up second gamepiece, and balance.")
public class MiddleScoreOnePickupBalance extends AutoBase {
    private final ForwardPixySubsystem forwardPixy;

    private final GamePiece gamePiece;
    
    public MiddleScoreOnePickupBalance(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm,
        ForwardPixySubsystem forwardPixy,
        GamePiece gamePiece
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);

        this.forwardPixy = forwardPixy;

        this.gamePiece = gamePiece;
    }
    
    public void init() {
        final double startingYOffset = getStartingYOffsetInches(
            autoConfiguration.getScoreGrid(), 
            autoConfiguration.getStartingNode()
        ) + (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);

        double pickUpYInches;
        switch (gamePiece) {
            case MIDDLE_LEFT_GAME_PIECE:
                pickUpYInches = 12;
                break;

            case MIDDLE_RIGHT_GAME_PIECE:
                pickUpYInches = -12;
                break;

            default:
                pickUpYInches = 0;
                break;
        }

        // Recenter offset to zero is the middle node
        final Pose2d initialPose = createPose2dInches(0, startingYOffset, 0);
        final Pose2d lineUpPose = createPose2dInches(24, 0, 0);
        final Pose2d chargeStationPose = createPose2dInches(134, 0, 0);
        final Pose2d driveOverPose = createPose2dInches(182, pickUpYInches, 0);
        final Pose2d pickUpPose = createPose2dInches(252, pickUpYInches, 0);
        final Pose2d secondLineUpPose = createPose2dInches(164, 0, 180);
        final Pose2d finalChargeStationPose = createPose2dInches(120, 0, 180);

        addCommands(new ResetOdometryCommand(drivetrain, initialPose));
        
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }
        
        addCommands(new ScoreCommand(
            () -> ScoreMode.CONE, 
            () -> autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.5,
            intake
        ));
        
        SwerveControllerCommand lineUpPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(1), 
            initialPose,
            lineUpPose,
            createRotation(180)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            lineUpPath.beforeStarting(new WaitCommand(0.5)),
            new CompleteScoreCommand(elevator, intake, arm)
        );
        addCommands(retractGroup);

        SwerveControllerCommand onChargePath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.chargeStationTrajectoryConfig.withStartAndEndVelocity(1, 0), 
            getLastEndingPose(),
            chargeStationPose,
            createRotation(180)
        );
        ParallelDeadlineGroup onChargeGroup = new ParallelDeadlineGroup(
            onChargePath,
            new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator)
        );
        addCommands(onChargeGroup);

        SwerveControllerCommand overChargePath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.snailTrajectoryConfig.withEndVelocity(0.75), 
            getLastEndingPose(),
            driveOverPose,
            createRotation(0)
        );

        ParallelDeadlineGroup overChargeGroup = new ParallelDeadlineGroup(
            overChargePath,
            new ArmOutCommand(arm)
        );
        addCommands(overChargeGroup);

        // Drive to approach and pick up the cone.
        Command pickupCommand = null;
        if (!Dashboard.getInstance().pixyCamBroken()) {
            pickupCommand = new GamePieceAlignmentCommand(
                () -> pickUpPose.getX(),
                forwardPixy, 
                drivetrain,
                intake
            );
            setLastEndingPose(pickUpPose);
        } else {
            pickupCommand = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig, 
                getLastEndingPose(),
                pickUpPose,
                createRotation(0)
            );
        }
        ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
            pickupCommand,
            new ElevatorPositionCommand(ElevatorPosition.GROUND_PICKUP, elevator),
            new IntakeInCommand(intake)
        );

        addCommands(pickUpGroup);

        addCommands(new ArmInCommand(arm));

        if (autoConfiguration.endChargeStation()) {
            SwerveControllerCommand secondLineupPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig, 
                getLastEndingPose(),
                secondLineUpPose,
                createRotation(180)
            );
            addCommands(secondLineupPath);

            SwerveControllerCommand chargeStationPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
                getLastEndingPose(),
                finalChargeStationPose,
                createRotation(180)
            );
            addCommands(chargeStationPath);

            addCommands(new ChargeStationBalanceCommand(drivetrain));
        }
    }
}
