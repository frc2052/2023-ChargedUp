// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class NeoSwerverModule extends SwerveModule {
    private final CANSparkMax  driveMotor;
    private final CANSparkMax  steerMotor;
    
    public NeoSwerverModule(
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
        driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

        driveMotor.setInverted(moduleGearConfiguration.isDriveInverted());

        checkError(
            "Failed to enable drive motor voltage compensation",
            driveMotor.enableVoltageCompensation(Constants.SwerveModule.MAX_VOLTAGE_VOLTS)
        );

        checkError(
            "Failed to set drive motor current limit",
            driveMotor.setSmartCurrentLimit((int) Constants.SwerveModule.DRIVE_CURRENT_LIMIT_AMPS)
        );

        checkError(
            "Failed to set drive motor periodic status frame rate",
            driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
            driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
            driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        );

        checkError(
            "Failed to set drive motor idle mode",
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        );

        RelativeEncoder encoder = driveMotor.getEncoder();
        encoder.setPositionConversionFactor(getDriveMetersPerTick());
        // Velocity derived from meters per tick per second
        encoder.setVelocityConversionFactor(getDriveMetersPerTick() / 60.0);

        /*
         * Steer Motor Initialization
         */
        steerMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

        checkError(
            "Failed to set steer motor periodic status frame 0 rate",
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100)
        );
        checkError(
            "Failed to set steer motor periodic status frame 1 rate",
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20)
        );
        checkError(
            "Failed to set steer motor periodic status frame 2 rate",
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        );

        checkError(
            "Failed to set steer motor idle mode",
            steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        );

        steerMotor.setInverted(!moduleGearConfiguration.isSteerInverted());

        checkError(
            "Failed to enable steer motor voltage compensation",
            steerMotor.enableVoltageCompensation(Constants.SwerveModule.MAX_VOLTAGE_VOLTS)
        );

        checkError(
            "Failed to set steer motor current limit",
            steerMotor.setSmartCurrentLimit((int) Constants.SwerveModule.DRIVE_CURRENT_LIMIT_AMPS)
        );

        
        // Sets the steer motor encoder to the absolute position of the CANCoder for startup orientation
        RelativeEncoder integratedEncoder = steerMotor.getEncoder();
        checkError(
            "Failed to set steer motor encoder conversion factor",
            integratedEncoder.setPositionConversionFactor(getSteerRadiansPerTick())
        );
        // Velocity derived from meters per tick per second
        checkError(
            "Failed to set steer motor encoder conversion factor",
            integratedEncoder.setVelocityConversionFactor(getSteerRadiansPerTick() / 60.0)
        );

        checkError(
            "Failed to set steer motor encoder position",
            integratedEncoder.setPosition(canCoder.getAbsolutePosition())
        );

        SparkMaxPIDController controller = steerMotor.getPIDController();
        checkError(
            "Failed to set steer motor PID proportional constant",
            controller.setP(Constants.SwerveModule.STEER_P)
        );
        checkError(
            "Failed to set steer motor PID integral constant",
            controller.setI(Constants.SwerveModule.STEER_I)
        );
        checkError(
            "Failed to set steer motor PID derivative constant",
            controller.setD(Constants.SwerveModule.STEER_D)
        );

        checkError(
            "Failed to set steer motor PID feedback device",
            controller.setFeedbackDevice(integratedEncoder)
        );
    }

    @Override
    SwerveModuleState getState() {
        // Both encoder values are automatically in correct units because of the position conversion factor
        return new SwerveModuleState(
            driveMotor.getEncoder().getPosition(),
            new Rotation2d(
                steerMotor.getEncoder().getPosition()
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

        // Set the motor to our desired voltage from a percentage of our max velocity multiplied by the nominal voltage
        driveMotor.setVoltage(desiredState.speedMetersPerSecond / Constants.SwerveModule.MAX_VELOCITY_METERS_PER_SECOND * Constants.SwerveModule.MAX_VOLTAGE_VOLTS);

        steerMotor.getPIDController().setReference(desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }
}
