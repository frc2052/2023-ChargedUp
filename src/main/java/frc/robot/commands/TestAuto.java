package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestAuto extends SequentialCommandGroup {
    private final DrivetrainSubsystem drivetrain;

    public TestAuto(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);

        addCommands(
            createSwerveCommand(
                new Pose2d(0, 0, new Rotation2d()), 
                new ArrayList<Translation2d>(),
                new Pose2d(2, 0, new Rotation2d()), 
                () -> Rotation2d.fromDegrees(180)
            ),
            createSwerveCommand(
                new Pose2d(2, 0, new Rotation2d()), 
                new ArrayList<Translation2d>(),
                new Pose2d(2, 1, new Rotation2d()), 
                () -> Rotation2d.fromDegrees(0)
            ),
            createSwerveCommand(
                new Pose2d(2, 1, new Rotation2d()), 
                new ArrayList<Translation2d>(),
                new Pose2d(0, 0, new Rotation2d()), 
                () -> Rotation2d.fromDegrees(-90)
            )
        );
    }

    private SwerveControllerCommand createSwerveCommand(
        Pose2d startPose, 
        List<Translation2d> midpoints,
        Pose2d endPose,
        Supplier<Rotation2d> rotation
    ) {
        TrajectoryConfig config = new TrajectoryConfig(
            2.5,
            1.5
        ).setKinematics(drivetrain.getKinematics());

        PIDController xyController = new PIDController(1, 0, 0);

        ProfiledPIDController thetaController = new ProfiledPIDController(
            10,
            0,
            0,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                midpoints,
                endPose,
                config
            ),
            drivetrain::getPosition,
            drivetrain.getKinematics(),
            xyController,
            xyController,
            thetaController,
            rotation,
            drivetrain::setModuleStates,
            drivetrain
        );
    }
}
