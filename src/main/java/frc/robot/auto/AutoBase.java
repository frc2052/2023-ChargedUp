// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public abstract class AutoBase extends SequentialCommandGroup {
    protected final DrivetrainSubsystem drivetrain;
    protected final ElevatorSubsystem elevator;
    protected final IntakeSubsystem intake;
    protected final ArmSubsystem arm;

    private Pose2d lastEndingPose;

    /** Creates a new Auto. */
    public AutoBase(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, IntakeSubsystem intake, ArmSubsystem arm) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;

        if (!this.elevator.elevatorZeroed()) {
            addCommands(new ZeroElevator(this.elevator));
        }

        init();
    }

    public abstract void init();

    public Pose2d getLastEndingPose() {
        return lastEndingPose;
    }

    public Pose2d createPose2dInches(double xInches, double yInches, double rotationDegrees) {
        return new Pose2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches), Rotation2d.fromDegrees(rotationDegrees));
    }

    public Translation2d createTranslation2dInches(double xInches, double yInches) {
        return new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches));
    }

    public double getLeftStartingYOffsetInches(Node startNode) {
        return -startNode.ordinal() * (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);
    }

    // Creates easy rotation 2d suppliers for the SwerveControllerCommands
    protected Supplier<Rotation2d> createRotation(double angleDegrees) {
        return () -> { return Rotation2d.fromDegrees(angleDegrees); };
    }

    protected SwerveControllerCommand createSwerveCommand(
        Pose2d startPose,
        Pose2d endPose,
        Supplier<Rotation2d> rotationSupplier
    ) {
        return createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            startPose, 
            new ArrayList<Translation2d>(), 
            endPose, 
            rotationSupplier
        );
    }

    protected SwerveControllerCommand createSwerveCommand(
        Pose2d startPose, 
        List<Translation2d> midpoints,
        Pose2d endPose,
        Supplier<Rotation2d> rotationSupplier
    ) {
        return createSwerveTrajectoryCommand(
            AutoTrajectoryConfig.defaultTrajectoryConfig, 
            startPose, 
            midpoints, 
            endPose, 
            rotationSupplier
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {
        return createSwerveTrajectoryCommand(
            trajectoryConfig, 
            startPose, 
            new ArrayList<Translation2d>(), 
            endPose, 
            rotationSupplier
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        List<Translation2d> midpointList,
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {
        lastEndingPose = endPose;

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                midpointList,
                endPose,
                trajectoryConfig.getTrajectoryConfig()
            ),
            drivetrain::getPosition,
            DrivetrainSubsystem.getKinematics(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(), 
            rotationSupplier,
            drivetrain::setModuleStates,
            drivetrain
        );
    }
}
