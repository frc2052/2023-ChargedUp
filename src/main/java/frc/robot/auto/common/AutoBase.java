// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.DrivetrainSubsystem;

public abstract class AutoBase extends SequentialCommandGroup {
    protected final AutoConfiguration autoConfiguration;
    protected final AutoRequirements autoRequirements;

    private Pose2d lastEndingPose;

    public AutoBase(
        AutoConfiguration autoConfiguration,
        AutoRequirements autoRequirements
    ) {
        this.autoConfiguration = autoConfiguration;
        this.autoRequirements = autoRequirements;

        // Automatically zero the elevator if not in a match.
        if (!DriverStation.isFMSAttached()) {
            if (!this.autoRequirements.getElevator().elevatorZeroed()) {
                addCommands(new ZeroElevator(this.autoRequirements.getElevator()));
            }
        }

        // Set zero as facing towards the driverstation.
        addCommands(new InstantCommand(autoRequirements.getDrivetrain()::zeroGyro, autoRequirements.getDrivetrain()));
    }

    public abstract void init();

    public Pose2d getLastEndingPose() {
        return lastEndingPose;
    }

    public void setLastEndingPose(Pose2d newEndingPose) {
        lastEndingPose = newEndingPose;
    }

    public Pose2d createPose2dInches(double xInches, double yInches, double rotationDegrees) {
        return new Pose2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches), Rotation2d.fromDegrees(rotationDegrees));
    }

    public Translation2d createTranslation2dInches(double xInches, double yInches) {
        return new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches));
    }

    public double getStartingYOffsetInches(Grid startGrid, Node startNode) {
        if (startGrid == Grid.LEFT_GRID) {
            // [0, 0] is located in the bottom left corner of the community with values becoming more negative farther right.
            return -startNode.ordinal() * (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);
        } else {
            // [0, 0] is located in the bottom right corner of the community with values becoming more positive farther left.
            return (2 - startNode.ordinal()) * (Constants.Auto.NODE_WIDTH_INCHES + Constants.Auto.NODE_DIVIDER_WIDTH_INCHES);
        }
    }

    // Creates easy rotation 2d suppliers for the SwerveControllerCommands.
    protected Supplier<Rotation2d> createRotation(double angleDegrees) {
        // Automatically adjust angles between positive and negative depending on which grid we're on.
        // Rotating counter clockwise is positive and clockwise is negative.
        double flipRotation = autoConfiguration.getStartingGrid() == Grid.RIGHT_GRID ? -1.0 : 1.0;

        return () -> { return Rotation2d.fromDegrees(angleDegrees * flipRotation); };
    }

    protected SwerveControllerCommand createSwerveCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {
        return createSwerveCommand(
            trajectoryConfig, 
            startPose, 
            new ArrayList<Translation2d>(), 
            endPose, 
            rotationSupplier
        );
    }

    protected SwerveControllerCommand createSwerveCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        List<Translation2d> midpointList,
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {
        lastEndingPose = endPose;

        // Automatically adjust y-coordinate depending on grid.
        // Positive is still left and negative is still right, but 0 is placed in each corner of the community.
        double flipYCoord = autoConfiguration.getStartingGrid() == Grid.RIGHT_GRID ? -1.0 : 1.0;

        List<Translation2d> adjustedMidpointList = new ArrayList<Translation2d>();
        for (Translation2d midpoint : midpointList) {
            adjustedMidpointList.add(new Translation2d(midpoint.getX(), midpoint.getY() * flipYCoord));
        }

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPose.getX(), startPose.getY() * flipYCoord, startPose.getRotation()),
                adjustedMidpointList,
                new Pose2d(endPose.getX(), endPose.getY() * flipYCoord, endPose.getRotation()),
                trajectoryConfig.getTrajectoryConfig()
            ),
            autoRequirements.getDrivetrain()::getPosition,
            DrivetrainSubsystem.getKinematics(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(), 
            rotationSupplier,
            autoRequirements.getDrivetrain()::setModuleStates,
            autoRequirements.getDrivetrain()
        );
    }
}
