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
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.auto.common.DashboardAutoRequirements;
import frc.robot.auto.common.ScorePickUpAutoBase;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeStopCommand;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, drive to grid and shoot without stopping, balance.")
@DashboardAutoRequirements(requirements = { Grid.class, Node.class, ChargeStation.class })
public class ScoreTwoBalanceAuto extends ScorePickUpAutoBase {
    public ScoreTwoBalanceAuto(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }
    
    public void init() {
        super.init();

        final Translation2d farChargeStationInterpolationPoint = createTranslation2dInches(108, -8);
        final Pose2d cableProtectorPoint = createPose2dInches(98, -4, 0);
        final Translation2d nearChargeStationInterpolationPoint = createTranslation2dInches(18, -6);
        final Pose2d lineUpPose = createPose2dInches(24, -80, 270);
        final Pose2d chargeStationPose = createPose2dInches(100, -80, 180);
       
        final AutoTrajectoryConfig driveBackTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 0, 1);
        final AutoTrajectoryConfig scoreTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 2, 1, 1, 2);
        final AutoTrajectoryConfig chargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 2, 0);

        // Driving back to cable protector.
        SwerveControllerCommand driveBackPath = createSwerveCommand(
            driveBackTrajectoryConfig,
            getLastEndingPose(), 
            List.of(farChargeStationInterpolationPoint), 
            cableProtectorPoint, 
            createRotation(180)
        );

        // Drive back to the grid.
        SwerveControllerCommand scorePath = createSwerveCommand(
            scoreTrajectoryConfig,
            getLastEndingPose(), 
            List.of(nearChargeStationInterpolationPoint), 
            lineUpPose, 
            createRotation(180)
        );
        addCommands(driveBackPath);

        ParallelDeadlineGroup driveBackAndScoreGroup = new ParallelDeadlineGroup(
            scorePath,
            new IntakeOutCommand(autoRequirements.getIntake()).beforeStarting(new WaitCommand(1.5))
        );
        addCommands(driveBackAndScoreGroup);

        addCommands(new IntakeStopCommand(autoRequirements.getIntake()));

        if (autoConfiguration.getChargeStation() == ChargeStation.BALANCE) {
            SwerveControllerCommand balancePath = createSwerveCommand(
                chargeStationTrajectoryConfig,
                getLastEndingPose(), 
                chargeStationPose, 
                createRotation(180)
            );
    
            addCommands(balancePath);

            addCommands(new ChargeStationBalanceCommand(autoRequirements.getDrivetrain()));
        }
    }
}
