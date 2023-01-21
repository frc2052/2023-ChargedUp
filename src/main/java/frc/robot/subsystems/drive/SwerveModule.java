// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public abstract class SwerveModule {
    protected final String debugName;

    protected final ModuleGearConfiguration moduleGearConfiguration;

    protected final CANCoder canCoder;

    public SwerveModule(
        String debugName,
        ModuleGearConfiguration moduleGearConfiguration,
        int canCoderChannel,
        Rotation2d steerOffset
    ) {
        this.debugName = debugName;

        this.moduleGearConfiguration = moduleGearConfiguration;

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
                Constants.SwerveModule.CAN_TIMEOUT_MS
            )
        );

        // Reduce CAN status frame rates
        checkError(
            "Failed to set CANCoder status frame period",
            canCoder.setStatusFramePeriod(
                CANCoderStatusFrame.SensorData,
                10,
                Constants.SwerveModule.CAN_TIMEOUT_MS
            )
        );
    }

    // TODO: Switch TICKS_PER_ROTATION constant
    public double getDriveMetersPerTick() {
        return (Math.PI * moduleGearConfiguration.getWheelDiameter() /
            Constants.Falcon500SwerveModule.TICKS_PER_ROTATION) * moduleGearConfiguration.getDriveReduction();
    }

    public double getSteerRadiansPerTick() {
        return (2.0 * Math.PI / Constants.Falcon500SwerveModule.TICKS_PER_ROTATION) *
            moduleGearConfiguration.getSteerReduction();
    }

    abstract SwerveModuleState getState();

    abstract void setState(double velocityMetersPerSecond, Rotation2d steerAngle);

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
