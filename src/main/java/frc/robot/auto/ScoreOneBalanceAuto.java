// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
import frc.robot.commands.intake.IntakeStopCommand;

@AutoDescription(description = "Score gamepiece, drive to pick up second gamepiece, and balance.")
@DashboardAutoRequirements(requirements = { Grid.class, Node.class, ChargeStation.class })
public class ScoreOneBalanceAuto extends ScorePickUpAutoBase {
    public ScoreOneBalanceAuto(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
        super(autoConfiguration, autoRequirements);
    }

    @Override
    public void init() {
        super.init(); 

        final Pose2d lineUpPose = createPose2dInches(170, -74, 180);
        final Pose2d chargeStationPose = createPose2dInches(60, -74, 180);

        final AutoTrajectoryConfig lineUpTrajectoryConfig = new AutoTrajectoryConfig(3, 2, 1, 4, 2, 0, 2);
        final AutoTrajectoryConfig chargeStationTrajectoryConfig = new AutoTrajectoryConfig(5, 3, 2, 3, 2, 2, 0);

        if (autoConfiguration.getChargeStation() == ChargeStation.BALANCE) {
            // Line up with the charge station.
            SwerveControllerCommand lineupPath = createSwerveCommand(
                lineUpTrajectoryConfig, 
                getLastEndingPose(), 
                lineUpPose, 
                createRotation(180)
            );

            addCommands(lineupPath);
        
            // Drive onto and balance on the charge station.
            SwerveControllerCommand onChargePath = super.createSwerveCommand(
                chargeStationTrajectoryConfig, 
                getLastEndingPose(), 
                chargeStationPose, 
                createRotation(180)
            );
        
            addCommands(onChargePath);
            
            addCommands(new IntakeStopCommand(autoRequirements.getIntake()));
            addCommands(new ChargeStationBalanceCommand(autoRequirements.getDrivetrain()));
        }
    }
}