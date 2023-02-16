// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor;
    
    // private final PIDController intakeController;
    private final SlewRateLimiter intakeLimiter;

    /** Creates a new Intake. */
    public IntakeSubsystem() {
        ErrorCode error;

        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
            true,
            // Limiting current once the peak current has been exceeded in amps
            Constants.Intake.INTAKE_CRUISE_CURRENT_AMPS,
            // Peak current in amps
            Constants.Intake.INTAKE_PEAK_CURRENT_THRESHOLD_AMPS,
            // Peak current duration before limiting in seconds
            Constants.Intake.INTAKE_PEAK_CURRENT_THRESHOLD_DURATION_SECONDS
        );

        intakeMotor = new TalonSRX(Constants.Intake.INTAKE_MOTOR_ID);
        intakeMotor.configFactoryDefault();

        if ((error = intakeMotor.configSupplyCurrentLimit(currentLimitConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError("Failed to configure intake motor current limit: " + error.toString(), true);
        }

        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(true);

        // intakeController = new PIDController(0.25, 0, 0);
        // intakeController.setTolerance(0.05);

        intakeLimiter = new SlewRateLimiter(4);
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(Constants.Dashboard.INTAKE_CURRENT_KEY, intakeMotor.getSupplyCurrent());

        // intakeMotor.set(
        //     ControlMode.PercentOutput, 
        //     intakeController.calculate(intakeMotor.getMotorOutputPercent())
        // );

        intakeLimiter.calculate(0);
    }

    public void intakeIn() {
        // intakeController.setSetpoint(0.75);
        intakeMotor.set(ControlMode.PercentOutput, intakeLimiter.calculate(0.75));
    }
    
    public void intakeOut() {
        // intakeController.setSetpoint(-1.0);
        intakeMotor.set(ControlMode.PercentOutput, intakeLimiter.calculate(-1.0));
    }

    public void stop() {
        // intakeController.setSetpoint(0.0);

        // Immediately stop intake and reset the slew rate limiter
        intakeLimiter.calculate(0.0);
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }
}