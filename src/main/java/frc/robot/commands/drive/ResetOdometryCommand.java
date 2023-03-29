// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetOdometryCommand extends InstantCommand {
    private final DrivetrainSubsystem drivetrain;

    private final Pose2d initialPose;
    private final Rotation2d initialRotation;

    public ResetOdometryCommand(DrivetrainSubsystem drivetrain, Pose2d initialPose) {
        this(drivetrain, initialPose, Rotation2d.fromDegrees(180));
    }

    public ResetOdometryCommand(DrivetrainSubsystem drivetrain, Pose2d initialPose, Rotation2d initialRotation) {
        this.drivetrain = drivetrain;

        this.initialPose = initialPose;
        this.initialRotation = initialRotation;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.resetOdometry(new Pose2d(initialPose.getX(), initialPose.getY(), initialRotation));
    }
}
