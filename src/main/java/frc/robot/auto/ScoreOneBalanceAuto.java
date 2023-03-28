// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Left side score gamepiece, drive to pick up gamepiece, drive to chargestation, and balance.
 */
public class ScoreOneBalanceAuto extends ScorePickUpAutoBase {
    /** Creates a new scoretwoandbalence. */
    public ScoreOneBalanceAuto(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);
    }

    @Override
    public void init() {
        super.init(); 

        Pose2d lineUpPose = createPose2dInches(170, -74, 180);
        Pose2d chargeStationPose = createPose2dInches(60, -74, 180);

        if (autoConfiguration.endChargeStation()) {
            // Drive to lineup w/ charge station
            SwerveControllerCommand lineupPath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.fastTurnDriveTrajectoryConfig, 
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