// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, drive to grid and shoot without stopping, balance.")
public class ScoreTwoBalanceAuto extends ScorePickUpAutoBase {
    public ScoreTwoBalanceAuto(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);
    }
    
    public void init() {
        super.init();

        Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(108, -8);
        Translation2d nearChargeStationInterpolationPoint = createTranslation2dInches(18, -6);
        Pose2d lineUpPose = createPose2dInches(24, -80, 270);
        Pose2d chargeStationPose = createPose2dInches(100, -80, 180);
       
        // Driving back to cable protector.
        SwerveControllerCommand driveBackPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig.withEndVelocity(1),
            getLastEndingPose(), 
            List.of(farChargeStationInterpolationPoint), 
            super.cableProtectorPoint, 
            createRotation(180)
        );

        // Drive back to the grid.
        SwerveControllerCommand scorePath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig.withStartAndEndVelocity(1, 2),
            getLastEndingPose(), 
            List.of(nearChargeStationInterpolationPoint), 
            lineUpPose, 
            createRotation(180)
        );
        addCommands(driveBackPath);

        ParallelDeadlineGroup driveBackAndScoreGroup = new ParallelDeadlineGroup(
            scorePath,
            new IntakeOutCommand(intake).beforeStarting(new WaitCommand(1.5))
        );
        addCommands(driveBackAndScoreGroup);

        addCommands(new IntakeStopCommand(intake));

        if (autoConfiguration.endChargeStation()) {
            SwerveControllerCommand balancePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig.withStartVelocity(2),
                getLastEndingPose(), 
                chargeStationPose, 
                createRotation(180)
            );
    
            addCommands(balancePath);

            addCommands(new ChargeStationBalanceCommand(drivetrain));
        }
    };
}
