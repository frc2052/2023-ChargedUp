package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto.Auto;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestAuto extends Auto {
    private final DrivetrainSubsystem drivetrain;

    public TestAuto(DrivetrainSubsystem drivetrain) {
        super(drivetrain);

        this.drivetrain = drivetrain;
    }

    @Override
    protected void init() {
        addCommands(
            createSwerveCommand(
                new Pose2d(0, 0, new Rotation2d()), 
                new ArrayList<Translation2d>(),
                new Pose2d(2, 0, new Rotation2d()), 
                Rotation2d.fromDegrees(0)
            )
            //,
            // createSwerveCommand(
            //     new Pose2d(2, 0, new Rotation2d()), 
            //     new ArrayList<Translation2d>(),
            //     new Pose2d(2, 1, new Rotation2d()), 
            //     () -> Rotation2d.fromDegrees(0)
            // ),
            // createSwerveCommand(
            //     new Pose2d(2, 1, new Rotation2d()), 
            //     new ArrayList<Translation2d>(),
            //     new Pose2d(0, 0, new Rotation2d()), 
            //     () -> Rotation2d.fromDegrees(-90)
            //)
        );
    }
}
