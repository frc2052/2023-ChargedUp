// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor;
    
    /** Creates a new Intake. */
    public IntakeSubsystem() {
        ErrorCode error;

        TalonSRXConfiguration intakeMotorConfiguration = new TalonSRXConfiguration();
        // Don't activate current limit until current exceeds 30A.
        intakeMotorConfiguration.peakCurrentLimit = 6;
        // For at least 100ms.
        intakeMotorConfiguration.peakCurrentDuration = 100;
        // Once current-limiting is actived, hold at 20A.
        intakeMotorConfiguration.continuousCurrentLimit = 5;

        intakeMotor = new TalonSRX(Constants.Intake.INTAKE_MOTOR_PWM_PORT);
        if ((error = intakeMotor.configAllSettings(intakeMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError("Failed to configure intake motor: " + error.toString(), true);
        }

        intakeMotor.enableCurrentLimit(true); 
    }

    public void intakeIn() {
        // Set the velocity of the intake motor by percent.
        intakeMotor.set(ControlMode.PercentOutput, .1);
    }
    
    public void intakeOut() {
        intakeMotor.set(ControlMode.PercentOutput, -.1);
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}