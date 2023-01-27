// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoBase<AutoTrajectoryConfig> extends SequentialCommandGroup {
  protected final DrivetrainSubsystem drivetrain;

  private Pose2d lastCreatedEndingPose;

  /** Creates a new autoBase. */
  public autoBase(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(this.drivetrain);
  }

  protected SwerveControllerCommand createSwerveCommand(
    TrajectoryConfig trajectoryConfig, 
    Pose2d startPose,
    List<Translation2d> midPoints, 
    Pose2d endPose,
    Supplier<Rotation2d> rotationSupplier
  ) {
    return new SwerveControllerCommand(
      TrajectoryGenerator.generateTrajectory(
        startPose,
        midPoints,
        endPose,
        trajectoryConfig
      ),
      drivetrain::getPosition,
      drivetrain.getKinematics(),
      new PIDController(0, 0, 0),
      new PIDController(0, 0, 0),
      new ProfiledPIDController(0, 0, 0, null),
      rotationSupplier,
      drivetrain::setModuleStates,
      drivetrain
    );
  }
}
