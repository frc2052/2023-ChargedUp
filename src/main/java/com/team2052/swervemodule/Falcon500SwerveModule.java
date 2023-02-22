// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2052.swervemodule;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Swerve module implementation for swerve module with Falcon500s
 */
public class Falcon500SwerveModule extends SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    
    private final double drivePositionConversionFactor;
    private final double driveVelocityConversionFactor;
    private final double steerPositionConversionFactor;

    public Falcon500SwerveModule(
        String debugName,
        ModuleConfiguration moduleConfiguration,
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel, 
        Rotation2d steerOffset
    ) {
        super(debugName, moduleConfiguration, canCoderChannel, steerOffset);

        /*
         * Drive Motor Initialization
         */
        // Conversion factor for switching between ticks and meters in terms of meters per tick
        drivePositionConversionFactor = (Math.PI * moduleConfiguration.getWheelDiameter() /
            SwerveConstants.Falcon500SwerveModule.TICKS_PER_ROTATION) * moduleConfiguration.getDriveReduction();
        // Conversion factor for switching between ticks and meters per second in terms of meters per second per tick
        driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.voltageCompSaturation = SwerveConstants.MAX_VOLTAGE_VOLTS;
        driveMotorConfiguration.supplyCurrLimit.currentLimit = SwerveConstants.DRIVE_CURRENT_LIMIT_AMPS;
        driveMotorConfiguration.supplyCurrLimit.enable = true;

        driveMotor = new TalonFX(driveMotorChannel);
        checkError("Failed to resotre drive motor factory defaults", driveMotor.configFactoryDefault());
        checkError("Failed to configure drive motor", driveMotor.configAllSettings(driveMotorConfiguration));

        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSensorPhase(true);
        driveMotor.setInverted(moduleConfiguration.isDriveInverted());
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // Reduce CAN status frame rates
        checkError(
            "Failed to set drive motor status frame period",
            driveMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                250,
                CAN_TIMEOUT_MS
            )
        );

        /*
         * Steer Motor Initialization
         */
        // Conversion factor for switching between ticks and radians in terms of radians per tick
        steerPositionConversionFactor = (2.0 * Math.PI / SwerveConstants.Falcon500SwerveModule.TICKS_PER_ROTATION) * 
            moduleConfiguration.getSteerReduction();

        TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.slot0.kP = SwerveConstants.Falcon500SwerveModule.STEER_MOTOR_P;
        steerMotorConfiguration.slot0.kI = SwerveConstants.Falcon500SwerveModule.STEER_MOTOR_I;
        steerMotorConfiguration.slot0.kD = SwerveConstants.Falcon500SwerveModule.STEER_MOTOR_D;

        steerMotorConfiguration.voltageCompSaturation = SwerveConstants.MAX_VOLTAGE_VOLTS;

        steerMotorConfiguration.supplyCurrLimit.currentLimit = SwerveConstants.STEER_CURRENT_LIMIT_AMPS;
        steerMotorConfiguration.supplyCurrLimit.enable = true;

        steerMotor = new TalonFX(steerMotorChannel);
        checkError("Failed to resotre steer motor factory defaults", steerMotor.configFactoryDefault());
        checkError("Failed to configure steer motor", steerMotor.configAllSettings(steerMotorConfiguration));

        steerMotor.enableVoltageCompensation(true);

        checkError(
            "Failed to set steer motor feedback sensor",
            steerMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                0,
                CAN_TIMEOUT_MS
            )
        );
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(moduleConfiguration.isSteerInverted());
        steerMotor.setNeutralMode(NeutralMode.Brake);

        // Sets the steer motor encoder to the absolute position of the CANCoder for startup orientation
        checkError(
            "Failed to set steer motor encoder position",
            steerMotor.setSelectedSensorPosition(
                Math.toRadians(canCoder.getAbsolutePosition()) / steerPositionConversionFactor,
                0,
                CAN_TIMEOUT_MS
            )
        );

        // Reduce CAN status frame rates
        checkError(
            "Failed to set steer motor status frame period",
            steerMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                250,
                CAN_TIMEOUT_MS
            )
        );
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            // Convert from ticks to meters per second using the predefined conversion factor
            driveMotor.getSelectedSensorVelocity() * driveVelocityConversionFactor,
            new Rotation2d(
                // Convert from ticks to radians using the predefined conversion factor
                steerMotor.getSelectedSensorPosition() * steerPositionConversionFactor
            )
        );
    }

    @Override
    public void setState(double velocityMetersPerSecond, Rotation2d steerAngle) {
        SwerveModuleState desiredState = new SwerveModuleState(velocityMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState = SwerveModuleState.optimize(
            desiredState,
            getState().angle
        );

        // Set the motor to our desired velocity as a percentage of our max velocity
        driveMotor.set(
            TalonFXControlMode.PercentOutput,
            desiredState.speedMetersPerSecond / getMaxVelocityMetersPerSecond(moduleConfiguration)
        );

        steerMotor.set(
            TalonFXControlMode.Position,
            desiredState.angle.getRadians() / steerPositionConversionFactor
        );
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getSelectedSensorPosition() * drivePositionConversionFactor,
            new Rotation2d(
                steerMotor.getSelectedSensorPosition() * steerPositionConversionFactor
            )
        );
    }

    public static double getMaxVelocityMetersPerSecond(ModuleConfiguration moduleConfiguration) {
        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        return SwerveConstants.Falcon500SwerveModule.FALCON500_ROUNDS_PER_MINUTE / 60 * moduleConfiguration.getDriveReduction() * 
            moduleConfiguration.getWheelDiameter() * Math.PI;
    }
}
