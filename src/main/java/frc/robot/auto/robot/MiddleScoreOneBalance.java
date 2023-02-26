// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.commands.arm.ArmInCommand;
import frc.robot.commands.drive.NewChargeStationBalanceCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleScoreOneBalance extends AutoBase {
    /** Creates a new MiddleScoreOneBalance. */
    public MiddleScoreOneBalance(
        boolean endChargeStation,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        super(drivetrain, elevator, intake, arm);

        Pose2d initialPose = createPose2dInches(0, 0, 0);
        Pose2d lineUpPose = createPose2dInches(24, 0, 0);
        Pose2d chargeStationPose = createPose2dInches(100, 0, 0);

        drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(0)));

        addCommands(new MidScoreCommand(elevator, arm));
        addCommands(new ArmInCommand(arm));
        addCommands(new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator));

        SwerveControllerCommand lineUpPath = createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.slowTrajectoryConfig.withEndVelocity(1), 
            initialPose,
            lineUpPose,
            createRotation(0)
        );

        addCommands(lineUpPath);
        
        if (endChargeStation) {
            SwerveControllerCommand chargePath = createSwerveTrajectoryCommand(
                AutoTrajectoryConfig.chargeStationTrajectoryConfig.withStartVelocity(1), 
                getLastEndingPose(),
                chargeStationPose,
                createRotation(0)
            );

            addCommands(chargePath);
            addCommands(new NewChargeStationBalanceCommand(drivetrain));
        }
    }
}
