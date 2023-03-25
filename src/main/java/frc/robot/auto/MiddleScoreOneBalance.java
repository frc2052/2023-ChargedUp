// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.auto.AutoFactory.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Middle score gamepiece, drive to chargestation, and balance.
 */
public class MiddleScoreOneBalance extends AutoBase {
    public MiddleScoreOneBalance(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm);
    }
    
    public void init() {
        Pose2d initialPose = createPose2dInches(
            0, 
            // Recenter offset to zero is the middle node
            getStartingYOffsetInches(
                autoConfiguration.getStartingGrid(), 
                autoConfiguration.getStartingNode()
            ) + (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES), 
            0
        );
        Translation2d lineUpMidpoint = createTranslation2dInches(
            -30, 
            0
        );
        Pose2d chargeStationPose = createPose2dInches(
            -150, 
            0, 
            0
        );

        addCommands(new InstantCommand(() -> { 
            drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180)));
        }, drivetrain));

        if (autoConfiguration.getStartingNode() == Node.MIDDLE_CUBE) {
            addCommands(new MidScoreCommand(elevator, arm));
        } else {
            addCommands(new TopScoreCommand(elevator, arm));
        }
        
        addCommands(new ScoreCommand(intake, arm, elevator, true).withTimeout(1));
        
        if (autoConfiguration.endChargeStation()) {
            SwerveControllerCommand chargePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig, 
                initialPose,
                List.of(lineUpMidpoint),
                chargeStationPose,
                createRotation(0)
            );
            addCommands(chargePath);

            addCommands(new ChargeStationBalanceCommand(drivetrain));
        }
    };
}
