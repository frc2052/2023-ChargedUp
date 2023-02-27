// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.red;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Left side score gamepiece, drive to pick up gamepiece, drive to grid, 
 * shoot without stopping, drive to charge station, and balance.
 */
public class RedLeftScoreTwoBalanceAuto extends ScorePickUpAuto {
    public RedLeftScoreTwoBalanceAuto(
        Grid startGrid,
        Node startNode,
        boolean endChargeStation,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(startGrid, startNode, endChargeStation, drivetrain, elevator, intake, arm);
    }
    
    public void init() {
        super.init();

        double flip = startGrid == Grid.LEFT_GRID ? -1.0 : 1.0;

        Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(108, 2 * flip);
        Translation2d nearChargeStationInterpolationPoint = createTranslation2dInches(18, 6 * flip);
        Pose2d lineUpPose = createPose2dInches(24, 80 * flip, 270);
        Pose2d chargeStationPose = createPose2dInches(100, 80 * flip, 180);
       
        // Driving back to grid
        SwerveControllerCommand driveBackPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withEndVelocity(2),
            getLastEndingPose(), 
            List.of(farChargeStationInterpolationPoint, nearChargeStationInterpolationPoint), 
            lineUpPose, 
            createRotation(180)
        );

        ParallelDeadlineGroup driveBackAndScoreGroup = new ParallelDeadlineGroup(
            driveBackPath,
            new IntakeOutCommand(intake).beforeStarting(new WaitCommand(4))
        );
        addCommands(driveBackAndScoreGroup);

        addCommands(new IntakeStopCommand(intake));

        if (endChargeStation) {
            SwerveControllerCommand balancePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig.withStartVelocity(2),
                getLastEndingPose(), 
                chargeStationPose, 
                createRotation(180)
            );
    
            addCommands(balancePath);

            addCommands(new ChargeStationBalanceCommand(drivetrain));
            addCommands(new RunCommand(() -> { drivetrain.xWheels(); }, drivetrain));
        }
    };
}
