// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Left side score gamepiece, drive to pick up gamepiece, drive to chargestation, and balance.
 */
public class RedScoreOneBalanceAuto extends ScorePickUpAuto {
    /** Creates a new scoretwoandbalence. */
    public RedScoreOneBalanceAuto(
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

    @Override
    public void init() {
        super.init(); 

        double flip = startGrid == Grid.LEFT_GRID ? -1.0 : 1.0;

        Pose2d lineUpPose = createPose2dInches(170, 74 * flip, 180);
        Pose2d chargeStationPose = createPose2dInches(60, 74 * flip, 180);

        if (endChargeStation) {
            // Drive to lineup w/ charge station
            SwerveControllerCommand lineupPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.fastTurnSlowDriveTrajectoryConfig, 
                getLastEndingPose(), 
                lineUpPose, 
                createRotation(180)
            );

            addCommands(lineupPath);
        
            //going on charge station
            SwerveControllerCommand onChargePath = super.createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
                getLastEndingPose(), 
                chargeStationPose, 
                createRotation(180)
            );
        
            addCommands(onChargePath);
            
            addCommands(new IntakeStopCommand(intake));
            addCommands(new ChargeStationBalanceCommand(drivetrain));
        }
    };
}