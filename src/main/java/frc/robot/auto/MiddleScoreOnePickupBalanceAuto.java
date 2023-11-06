// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.GamePiece;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.arm.ArmOutCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.drive.GamePieceAlignmentCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.commands.score.CompleteScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

@AutoDescription(description = "Score gamepiece, drive over charge station to pick up second gamepiece, and balance.")
@DashboardAutoRequirements(requirements = { Node.class, GamePiece.class, ChargeStation.class })
public class MiddleScoreOnePickupBalanceAuto extends AutoBase {
    public MiddleScoreOnePickupBalanceAuto(
        AutoConfiguration autoConfiguration,
        AutoRequirements autoRequirements
    ) {
        super(autoConfiguration, autoRequirements);
    }
    
    public void init() {
        // final double startingYOffset = getStartingYOffsetInches(
        //     Grid.MIDDLE_GRID, 
        //     autoConfiguration.getStartingNode()
        // ) + (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);

        double pickUpYInches;
        switch (autoConfiguration.getGamePiece()) {
            case MIDDLE_LEFT_GAME_PIECE:
                pickUpYInches = 42;
                break;

            case MIDDLE_RIGHT_GAME_PIECE:
                pickUpYInches = -24;
                break;

            default:
                pickUpYInches = 0;
                break;
        }

        double startingY = 0;
        switch (autoConfiguration.getStartingNode()) {
            case MIDDLE_CUBE:
                startingY = 0;
                break;

            case RIGHT_CONE:
                startingY = -(Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);
                break;

            case LEFT_CONE:
                startingY = Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES;
                break;
        }

        // Recenter offset to zero is the middle node
        final Pose2d initialPose = createPose2dInches(autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 12.25 : 0, startingY, 0);
        final Pose2d lineUpPose = createPose2dInches(24, -6, 0);
        final Pose2d chargeStationPose = createPose2dInches(134, pickUpYInches / 2, 0);
        final Pose2d driveOverPose = createPose2dInches(190, pickUpYInches, 0);
        final Pose2d pickUpPose = createPose2dInches(268, pickUpYInches, 0);
        final Pose2d secondLineUpPose = createPose2dInches(164, autoConfiguration.getGamePiece() == GamePiece.MIDDLE_LEFT_GAME_PIECE ? 12 : -12, 180);
        final Pose2d finalChargeStationPose = createPose2dInches(120, autoConfiguration.getGamePiece() == GamePiece.MIDDLE_LEFT_GAME_PIECE ? 12 : -12, 180);

        final AutoTrajectoryConfig retractTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 2);
        final AutoTrajectoryConfig chargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 4, 2, 3, 2, 2, 0.5);
        final AutoTrajectoryConfig driveOverTrajectoryConfig = new AutoTrajectoryConfig(1, 1.5, 0.5, 3, 2.5, 0.5, 0.4);
        final AutoTrajectoryConfig pickUpTrajectoryConfig = new AutoTrajectoryConfig(3, 3, 1, 4, 2, 0.4, 0);
        final AutoTrajectoryConfig lineUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 3, 2, 0, 2);
        final AutoTrajectoryConfig rechargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 2, 0);

        addCommands(new ResetOdometryCommand(initialPose));
        
        // Initial elevator score command.
        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new ParallelCommandGroup(
                new ElevatorPositionCommand(87500, autoRequirements.getElevator()),
                new ArmOutCommand(autoRequirements.getArm()).beforeStarting(new WaitCommand(0.2))
            ));
        } else {
            addCommands(new TopScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm()));
            addCommands(new IntakeOutCommand(autoRequirements.getIntake()));
            addCommands(new WaitCommand(0.25));
        }
        // addCommands(new InstantCommand(() -> autoRequirements.getIntake().setScoreMode(autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? ScoreMode.CUBE : ScoreMode.CONE)));
        // addCommands(new ScoreCommand(() -> autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.25, autoRequirements.getIntake()));
        
        // addCommands(new ParallelCommandGroup(
        //     new ElevatorPositionCommand(87500, autoRequirements.getElevator()),
        //     new ArmOutCommand(autoRequirements.getArm()).beforeStarting(new WaitCommand(0.2))
        // ));

        SwerveControllerCommand lineUpPath = createSwerveCommand(
            retractTrajectoryConfig, 
            initialPose,
            lineUpPose,
            createRotation(180)
        );
        ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
            lineUpPath.beforeStarting(new WaitCommand(0.125)),
            new CompleteScoreCommand(autoRequirements.getElevator(), autoRequirements.getIntake(), autoRequirements.getArm())
        );
        addCommands(retractGroup);

        SwerveControllerCommand onChargePath = createSwerveCommand(
            chargeStationTrajectoryConfig, 
            getLastEndingPose(),
            chargeStationPose,
            createRotation(180)
        );
        ParallelDeadlineGroup onChargeGroup = new ParallelDeadlineGroup(
            onChargePath,
            new ElevatorPositionCommand(20000, autoRequirements.getElevator())
        );
        addCommands(onChargeGroup);

        SwerveControllerCommand overChargePath = createSwerveCommand(
            driveOverTrajectoryConfig, 
            getLastEndingPose(),
            driveOverPose,
            createRotation(5 * (autoConfiguration.getGamePiece() == GamePiece.MIDDLE_LEFT_GAME_PIECE ? 1 : -1))
        );

        ParallelDeadlineGroup overChargeGroup = new ParallelDeadlineGroup(
            overChargePath,
            new ElevatorPositionCommand(10000, autoRequirements.getElevator()),
            new ArmOutCommand(autoRequirements.getArm())
        );
        addCommands(overChargeGroup);

        // Drive to approach and pick up the cone.
        Command pickupCommand = null;
        if (!Dashboard.getInstance().pixyCamBroken()) {
            pickupCommand = new GamePieceAlignmentCommand(
                () -> pickUpPose.getX(),
                autoRequirements.getDrivetrain(),
                autoRequirements.getForwardPixy()
            );
            setLastEndingPose(pickUpPose);
        } else {
            pickupCommand = createSwerveCommand(
                pickUpTrajectoryConfig, 
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

        if (autoConfiguration.getChargeStation() == ChargeStation.BALANCE) {
            SwerveControllerCommand secondLineupPath = createSwerveCommand(
                lineUpTrajectoryConfig, 
                getLastEndingPose(),
                secondLineUpPose,
                createRotation(180)
            );
            ParallelDeadlineGroup lineupGroup = new ParallelDeadlineGroup(
                secondLineupPath
                // new ElevatorPositionCommand(30000, autoRequirements.getElevator()).beforeStarting(new WaitCommand(0.25))
            );
            addCommands(lineupGroup);

            SwerveControllerCommand chargeStationPath = createSwerveCommand(
                rechargeStationTrajectoryConfig, 
                getLastEndingPose(),
                finalChargeStationPose,
                createRotation(180)
            );
            ParallelDeadlineGroup chargeStationGroup = new ParallelDeadlineGroup(
                chargeStationPath,
                new ElevatorPositionCommand(0, autoRequirements.getElevator()).beforeStarting(new WaitCommand(0.25))
            );
            addCommands(chargeStationGroup);

            addCommands(new IntakeStopCommand(autoRequirements.getIntake()));

            ParallelCommandGroup finalScoreGroup = new ParallelCommandGroup(
                new ChargeStationBalanceCommand(autoRequirements.getDrivetrain()),
                new WaitUntilCommand(autoRequirements.getDrivetrain()::getBalanced).andThen(
                    new ParallelCommandGroup(
                        new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, autoRequirements.getElevator()),
                        new InstantCommand(() -> { autoRequirements.getIntake().setScoreMode(ScoreMode.CUBE); }).andThen(
                            new IntakeOutCommand(autoRequirements.getIntake()).beforeStarting(new WaitCommand(0.875))
                        )
                    ).andThen(
                        new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, autoRequirements.getElevator()),
                        new IntakeStopCommand(autoRequirements.getIntake())
                    )
                )
            );

            addCommands(finalScoreGroup);
        }
    }
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.auto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.auto.common.AutoBase;
// import frc.robot.auto.common.AutoConfiguration;
// import frc.robot.auto.common.AutoDescription;
// import frc.robot.auto.common.AutoRequirements;
// import frc.robot.auto.common.AutoTrajectoryConfig;
// import frc.robot.auto.common.DashboardAutoRequirements;
// import frc.robot.auto.common.AutoFactory.ChargeStation;
// import frc.robot.auto.common.AutoFactory.GamePiece;
// import frc.robot.auto.common.AutoFactory.Node;
// import frc.robot.commands.arm.ArmInCommand;
// import frc.robot.commands.arm.ArmOutCommand;
// import frc.robot.commands.drive.ChargeStationBalanceCommand;
// import frc.robot.commands.drive.GamePieceAlignmentCommand;
// import frc.robot.commands.drive.ResetOdometryCommand;
// import frc.robot.commands.elevator.ElevatorPositionCommand;
// import frc.robot.commands.intake.IntakeInCommand;
// import frc.robot.commands.intake.IntakeStopCommand;
// import frc.robot.commands.score.CompleteScoreCommand;
// import frc.robot.commands.score.MidScoreCommand;
// import frc.robot.commands.score.ScoreCommand;
// import frc.robot.commands.score.TopScoreCommand;
// import frc.robot.io.Dashboard;
// import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
// import frc.robot.subsystems.IntakeSubsystem.ScoreMode;

// @AutoDescription(description = "Score gamepiece, drive over charge station to pick up second gamepiece, and balance.")
// @DashboardAutoRequirements(requirements = { Node.class, GamePiece.class, ChargeStation.class })
// public class MiddleScoreOnePickupBalanceAuto extends AutoBase {
//     public MiddleScoreOnePickupBalanceAuto(
//         AutoConfiguration autoConfiguration,
//         AutoRequirements autoRequirements
//     ) {
//         super(autoConfiguration, autoRequirements);
//     }
    
//     public void init() {
//         // final double startingYOffset = getStartingYOffsetInches(
//         //     Grid.MIDDLE_GRID, 
//         //     autoConfiguration.getStartingNode()
//         // ) + (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);

//         double pickUpYInches;
//         switch (autoConfiguration.getGamePiece()) {
//             case MIDDLE_LEFT_GAME_PIECE:
//                 pickUpYInches = 24;
//                 break;

//             case MIDDLE_RIGHT_GAME_PIECE:
//                 pickUpYInches = -24;
//                 break;

//             default:
//                 pickUpYInches = 0;
//                 break;
//         }

//         // Recenter offset to zero is the middle node
//         final Pose2d initialPose = createPose2dInches(12.25, 0, 0);
//         final Pose2d lineUpPose = createPose2dInches(24, 0, 0);
//         final Pose2d chargeStationPose = createPose2dInches(134, 0, 0);
//         final Pose2d driveOverPose = createPose2dInches(182, pickUpYInches, 0);
//         final Pose2d pickUpPose = createPose2dInches(258, pickUpYInches, 0);
//         final Pose2d secondLineUpPose = createPose2dInches(164, 0, 180);
//         final Pose2d finalChargeStationPose = createPose2dInches(120, 0, 180);

//         final AutoTrajectoryConfig retractTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 0, 1);
//         final AutoTrajectoryConfig chargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 1, 0.5);
//         final AutoTrajectoryConfig driveOverTrajectoryConfig = new AutoTrajectoryConfig(1, 1.5, 0.5, 2, 2, 0.5, 2);
//         final AutoTrajectoryConfig pickUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 2, 0);
//         final AutoTrajectoryConfig lineUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 3, 3, 0, 2);
//         final AutoTrajectoryConfig rechargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 2, 0);

//         addCommands(new ResetOdometryCommand(autoRequirements.getDrivetrain(), initialPose));
        
//         // Initial elevator score command.
//         if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
//             addCommands(new MidScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm()));
//         } else {
//             addCommands(new TopScoreCommand(autoRequirements.getElevator(), autoRequirements.getArm()));
//         }
//         addCommands(new InstantCommand(() -> autoRequirements.getIntake().setScoreMode(autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? ScoreMode.CUBE : ScoreMode.CONE)));
//         addCommands(new ScoreCommand(() -> autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE ? 0 : 0.25, autoRequirements.getIntake()));
        
//         SwerveControllerCommand lineUpPath = createSwerveCommand(
//             retractTrajectoryConfig, 
//             initialPose,
//             lineUpPose,
//             createRotation(180)
//         );
//         ParallelDeadlineGroup retractGroup = new ParallelDeadlineGroup(
//             lineUpPath.beforeStarting(new WaitCommand(0.25)),
//             new IntakeStopCommand(autoRequirements.getIntake()),
//             new ElevatorPositionCommand(40000, autoRequirements.getElevator())
//         );
//         addCommands(retractGroup);

//         SwerveControllerCommand onChargePath = createSwerveCommand(
//             chargeStationTrajectoryConfig, 
//             getLastEndingPose(),
//             chargeStationPose,
//             createRotation(180)
//         );
//         addCommands(onChargePath);

//         SwerveControllerCommand overChargePath = createSwerveCommand(
//             driveOverTrajectoryConfig, 
//             getLastEndingPose(),
//             driveOverPose,
//             createRotation(0)
//         );

//         ParallelDeadlineGroup overChargeGroup = new ParallelDeadlineGroup(
//             overChargePath,
//             new ElevatorPositionCommand(10000, autoRequirements.getElevator())
//         );
//         addCommands(overChargeGroup);

//         // Drive to approach and pick up the cone.
//         Command pickupCommand = null;
//         if (!Dashboard.getInstance().pixyCamBroken()) {
//             pickupCommand = new GamePieceAlignmentCommand(
//                 () -> pickUpPose.getX(),
//                 autoRequirements.getDrivetrain(),
//                 autoRequirements.getForwardPixy(),
//                 autoRequirements.getIntake()
//             );
//             setLastEndingPose(pickUpPose);
//         } else {
//             pickupCommand = createSwerveCommand(
//                 pickUpTrajectoryConfig, 
//                 getLastEndingPose(),
//                 pickUpPose,
//                 createRotation(0)
//             );
//         }
//         ParallelDeadlineGroup pickUpGroup = new ParallelDeadlineGroup(
//             pickupCommand,
//             new ElevatorPositionCommand(ElevatorPosition.GROUND_CONE_PICKUP, autoRequirements.getElevator()),
//             new IntakeInCommand(autoRequirements.getIntake())
//         );
//         addCommands(pickUpGroup);

//         addCommands(new ArmInCommand(autoRequirements.getArm()));

//         if (autoConfiguration.getChargeStation() == ChargeStation.BALANCE) {
//             SwerveControllerCommand secondLineupPath = createSwerveCommand(
//                 lineUpTrajectoryConfig, 
//                 getLastEndingPose(),
//                 secondLineUpPose,
//                 createRotation(180)
//             );
//             ParallelDeadlineGroup secondLineupGroup = new ParallelDeadlineGroup(
//                 secondLineupPath, 
//                 new ElevatorPositionCommand(40000, autoRequirements.getElevator())
//             );
//             addCommands(secondLineupGroup);

//             SwerveControllerCommand chargeStationPath = createSwerveCommand(
//                 rechargeStationTrajectoryConfig, 
//                 getLastEndingPose(),
//                 finalChargeStationPose,
//                 createRotation(180)
//             );
//             addCommands(chargeStationPath);

//             addCommands(new ChargeStationBalanceCommand(autoRequirements.getDrivetrain()));
//         }
//     }
// }
