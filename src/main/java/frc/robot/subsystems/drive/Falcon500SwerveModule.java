// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class Falcon500SwerveModule extends SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    
    public Falcon500SwerveModule(
        String debugName,
        ModuleGearConfiguration moduleGearConfiguration,
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel, 
        Rotation2d steerOffset
    ) {
        super(debugName, moduleGearConfiguration, canCoderChannel, steerOffset);

        /*
         * Drive Motor Initialization
         */
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.voltageCompSaturation = Constants.SwerveModule.MAX_VOLTAGE_VOLTS;
        driveMotorConfiguration.supplyCurrLimit.currentLimit = Constants.SwerveModule.DRIVE_CURRENT_LIMIT_AMPS;
        driveMotorConfiguration.supplyCurrLimit.enable = true;

        driveMotor = new TalonFX(driveMotorChannel);
        checkError(
            "Failed to configure drive motor",
            driveMotor.configAllSettings(driveMotorConfiguration)
        );
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSensorPhase(true);
        driveMotor.setInverted(moduleGearConfiguration.isDriveInverted());
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // Reduce CAN status frame rates
        checkError(
            "Failed to set drive motor status frame period",
            driveMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                250,
                Constants.SwerveModule.CAN_TIMEOUT_MS
            )
        );

        /*
         * Steer Motor Initialization
         */
        TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.slot0.kP = Constants.SwerveModule.STEER_P;
        steerMotorConfiguration.slot0.kI = Constants.SwerveModule.STEER_I;
        steerMotorConfiguration.slot0.kD = Constants.SwerveModule.STEER_D;

        steerMotorConfiguration.voltageCompSaturation = Constants.SwerveModule.MAX_VOLTAGE_VOLTS;

        steerMotorConfiguration.supplyCurrLimit.currentLimit = Constants.SwerveModule.STEER_CURRENT_LIMIT_AMPS;
        steerMotorConfiguration.supplyCurrLimit.enable = true;

        steerMotor = new TalonFX(steerMotorChannel);
        checkError(
            "Failed to configure steer motor",
            steerMotor.configAllSettings(steerMotorConfiguration)
        );

        steerMotor.enableVoltageCompensation(true);

        checkError(
            "Failed to set steer motor feedback sensor",
            steerMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                0,
                Constants.SwerveModule.CAN_TIMEOUT_MS
            )
        );
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(moduleGearConfiguration.isSteerInverted());
        steerMotor.setNeutralMode(NeutralMode.Brake);

        // Sets the steer motor encoder to the absolute position of the CANCoder for startup orientation
        checkError(
            "Failed to set steer motor encoder position",
            steerMotor.setSelectedSensorPosition(
                Math.toRadians(canCoder.getAbsolutePosition()) / getSteerRadiansPerTick(),
                0,
                Constants.SwerveModule.CAN_TIMEOUT_MS
            )
        );

        // Reduce CAN status frame rates
        checkError(
            "Failed to set steer motor status frame period",
            steerMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                250,
                Constants.SwerveModule.CAN_TIMEOUT_MS
            )
        );
    }

    @Override
    SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorPosition() * getDriveMetersPerTick(),
            new Rotation2d(
                steerMotor.getSelectedSensorPosition() * getSteerRadiansPerTick()
            )
        );
    }

    @Override
    void setState(double velocityMetersPerSecond, Rotation2d steerAngle) {
        SwerveModuleState desiredState = new SwerveModuleState(velocityMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState = SwerveModuleState.optimize(
            desiredState,
            getState().angle
        );

        SmartDashboard.putNumber(debugName + ": Speed", desiredState.speedMetersPerSecond);

        SmartDashboard.putNumber(debugName + ": Rotation", desiredState.angle.getRadians());

        // Set the motor to our desired velocity as a percentage of our max velocity
        driveMotor.set(
            TalonFXControlMode.PercentOutput,
            desiredState.speedMetersPerSecond / Constants.SwerveModule.MAX_VELOCITY_METERS_PER_SECOND
        );

        steerMotor.set(
            TalonFXControlMode.Position,
            desiredState.angle.getRadians() / getSteerRadiansPerTick()
        );
    }
}
