// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2052.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Swerve module implementation for swerve module with Neos
 */
public class NeoSwerverModule extends SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    
    private final double maxVelocityMetersPerSecond;
    private final double maxAngularVelocityRadiansPerSecond;

    private static final double STEER_P = 1.0;
    private static final double STEER_I = 0.0;
    private static final double STEER_D = 0.1;

    public NeoSwerverModule(
        String debugName, 
        ModuleConfiguration moduleConfiguration,
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel,
        Rotation2d steerOffset
    ) {
        super(debugName, moduleConfiguration, canCoderChannel, steerOffset);

        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * By default this value is setup for a Mk3 standard module using Falcon500s to drive.
         * An example of this constant for a Mk4 L2 module with NEOs to drive is:
         * 5880.0 (RPM) / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        maxVelocityMetersPerSecond = 5676 / 60 * moduleConfiguration.getDriveReduction() * moduleConfiguration.getWheelDiameter() * Math.PI;
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

        maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond / Math.hypot(
            Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 
            Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
        );

        /*
         * Drive Motor Initialization
         */
        driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        
        // Reduce CAN status frame rates
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
        driveMotor.setInverted(moduleConfiguration.isDriveInverted());

        checkError(
            "Failed to enable drive motor voltage compensation",
            driveMotor.enableVoltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS)
        );
        checkError(
            "Failed to set drive motor current limit",
            driveMotor.setSmartCurrentLimit((int) SwerveConstants.DRIVE_CURRENT_LIMIT_AMPS)
        );

        // Drive Motor encoder initialization
        RelativeEncoder encoder = driveMotor.getEncoder();

        // Conversion factor for switching between ticks and meters in terms of meters per tick
        double drivePositionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * 
            moduleConfiguration.getDriveReduction();
        
        checkError(
            "Failed to set drive motor encoder conversion factors",
            // Set the position conversion factor so the encoder will automatically convert ticks to meters
            encoder.setPositionConversionFactor(drivePositionConversionFactor),
            // Velocity of the encoder in meters per second
            encoder.setVelocityConversionFactor(drivePositionConversionFactor / 60.0)
        );

        /*
         * Steer Motor Initialization
         */
        steerMotor = new CANSparkMax(steerMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Reduce CAN status frame rates
        checkError(
            "Failed to set steer motor periodic status frame rate",
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        );

        checkError(
            "Failed to set steer motor idle mode",
            steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        );
        steerMotor.setInverted(!moduleConfiguration.isSteerInverted());

        checkError(
            "Failed to enable steer motor voltage compensation",
            steerMotor.enableVoltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS)
        );

        checkError(
            "Failed to set steer motor current limit",
            steerMotor.setSmartCurrentLimit((int) SwerveConstants.STEER_CURRENT_LIMIT_AMPS)
        );

        // Steer Motor encoder initialization
        RelativeEncoder integratedEncoder = steerMotor.getEncoder();

        // Conversion factor for switching between ticks and radians in terms of radians per tick
        double steerPositionConversionFactor = 2.0 * Math.PI * moduleConfiguration.getSteerReduction();

        checkError(
            "Failed to set drive motor encoder conversion factors",
            // Set the position conversion factor so the encoder will automatically convert ticks to radians
            integratedEncoder.setPositionConversionFactor(steerPositionConversionFactor),
            // Velocity of the encoder in radians per second
            integratedEncoder.setVelocityConversionFactor(steerPositionConversionFactor / 60.0)
        );

        // Sets the steer motor encoder to the absolute position of the CANCoder for startup orientation
        checkError(
            "Failed to set steer motor encoder position",
            integratedEncoder.setPosition(canCoder.getAbsolutePosition())
        );

        SparkMaxPIDController controller = steerMotor.getPIDController();
        checkError(
            "Failed to set steer motor PID proportional constant",
            controller.setP(STEER_P),
            controller.setI(STEER_I),
            controller.setD(STEER_D)
        );

        checkError(
            "Failed to set steer motor PID feedback device",
            controller.setFeedbackDevice(integratedEncoder)
        );
    }

    @Override
    public SwerveModuleState getState() {
        // Both encoder values are automatically in units of meters per second and
        // radians because of the position and velocity conversion factors
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(
                steerMotor.getEncoder().getPosition() % (2.0 * Math.PI)
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

        // Set the motor to the desired voltage using a
        // percentage of the max velocity multiplied by the nominal voltage
        driveMotor.setVoltage(
            desiredState.speedMetersPerSecond / maxVelocityMetersPerSecond * SwerveConstants.MAX_VOLTAGE_VOLTS
        );

        SmartDashboard.putNumber(debugName + ": Speed", desiredState.speedMetersPerSecond);

        steerMotor.getPIDController().setReference(
            desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition
        );
        
        SmartDashboard.putNumber(debugName + ": Rotation", desiredState.angle.getDegrees());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(
                steerMotor.getEncoder().getPosition()
            )
        );
    }
}
