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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public abstract class AutoBase extends SequentialCommandGroup {
    protected final AutoConfiguration autoConfiguration;
    protected final DrivetrainSubsystem drivetrain;
    protected final ElevatorSubsystem elevator;
    protected final IntakeSubsystem intake;
    protected final ArmSubsystem arm;

    private Pose2d lastEndingPose;

    /** Creates a new Auto. */
    public AutoBase(
        AutoConfiguration autoConfiguration,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm
    ) {
        this.autoConfiguration = autoConfiguration;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;

        if (!DriverStation.isFMSAttached()) {
            if (!this.elevator.elevatorZeroed()) {
                addCommands(new ZeroElevator(this.elevator));
            }
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

    public double getLeftStartingYOffsetInches(Grid startGrid, Node startNode) {
        if (startGrid == Grid.LEFT_GRID) {
            return startNode.ordinal() * (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);
        } else {
            return (2 - startNode.ordinal()) * (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);
        }
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

        double flipYCoord = autoConfiguration.getStartingGrid() == Grid.LEFT_GRID ? -1.0 : 1.0;

        List<Translation2d> adjustedMidpointList = new ArrayList<Translation2d>();
        for (Translation2d midpoint : midpointList) {
            adjustedMidpointList.add(new Translation2d(midpoint.getX(), midpoint.getY()));
        }
        
        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPose.getX(), startPose.getY() * flipYCoord, startPose.getRotation()),
                adjustedMidpointList,
                new Pose2d(endPose.getX(), endPose.getY() * flipYCoord, endPose.getRotation()),
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
