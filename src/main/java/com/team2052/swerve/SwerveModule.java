// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2052.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Shared elements of swerve modules regardless of motor type
 */
public abstract class SwerveModule {
    protected static final int CAN_TIMEOUT_MS = 250;

    protected final String debugName;

    protected final ModuleConfiguration moduleConfiguration;

    protected final CANCoder canCoder;

    protected final double maxVelocityMetersPerSecond;

    public SwerveModule(
        String debugName,
        ModuleConfiguration moduleConfiguration,
        int canCoderChannel,
        Rotation2d steerOffset
    ) {
        this.debugName = debugName;

        this.moduleConfiguration = moduleConfiguration;

        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * By default this value is setup for a Mk3 standard module using Falcon500s to drive.
         * An example of this constant for a Mk4 L2 module with NEOs to drive is:
         * 5880.0 (RPM) / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        maxVelocityMetersPerSecond = 6380.0 / 60.0 * moduleConfiguration.getDriveReduction() * moduleConfiguration.getWheelDiameter() * Math.PI;
        /*
         * This would be the theoretical maximum angular velocity of the robot in radians per second.
         * 
         *  maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond / Math.hypot(
         *      Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 
         *      Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
         *  );
         *
         * This would be a measure of how fast the robot can rotate in place.
         */

        /*
         * CANCoder Initialization
         */
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.magnetOffsetDegrees = steerOffset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        canCoder = new CANCoder(canCoderChannel);
        checkError(
            "Failed to configure CANCoder",
            canCoder.configAllSettings(
                canCoderConfiguration,
                CAN_TIMEOUT_MS
            )
        );

        // Reduce CAN status frame rates
        checkError(
            "Failed to set CANCoder status frame period",
            canCoder.setStatusFramePeriod(
                CANCoderStatusFrame.SensorData,
                10,
                CAN_TIMEOUT_MS
            )
        );
    }

    public abstract SwerveModuleState getState();

    public abstract void setState(double velocityMetersPerSecond, Rotation2d steerAngle);

    /**
     * Similar to the {@link #getState()} method, but returns a position and rotation
     * rather than a velocity and rotation
     * 
     * @return Swerve module position in (meters, Rotation2D)
     */
    public abstract SwerveModulePosition getPosition();

    @SuppressWarnings("unchecked")
    protected <E> void checkError(String message, E... errors) {
        for (E error : errors) {
            if (error != REVLibError.kOk || error != ErrorCode.OK) {
                DriverStation.reportError(
                    message + " on [" + debugName + "] module: " + error.toString(),
                    false
                );
            }
        }
    }
}
