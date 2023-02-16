// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor;
    
    private boolean hasGamePiece = false;
    private final Timer timer;

    /** Creates a new Intake. */
    public IntakeSubsystem() {
        ErrorCode error = ErrorCode.OK;

        TalonSRXConfiguration intakeMotorConfiguration = new TalonSRXConfiguration();
        // Don't activate current limit until current exceeds 20A.
        intakeMotorConfiguration.peakCurrentLimit = 10;
        // For at least 100ms.
        intakeMotorConfiguration.peakCurrentDuration = 100;
        // Once current-limiting is actived, hold at 2A.
        intakeMotorConfiguration.continuousCurrentLimit = 2;

        intakeMotor = new TalonSRX(Constants.Intake.INTAKE_MOTOR_ID);
        intakeMotor.configFactoryDefault();
        if ((error = intakeMotor.configAllSettings(intakeMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError("Failed to configure intake motor: " + error.toString(), true);
        }
        intakeMotor.setInverted(true);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.enableCurrentLimit(false); 

        timer = new Timer();
    }

    @Override
    public void periodic() {
        if (intakeMotor.getSupplyCurrent() > 20) {
            timer.start();
        } else {
            timer.stop();
            timer.reset();
        }

        if (timer.get() > 0.1) {
            hasGamePiece = true;
            timer.stop();
            timer.reset();
        }

        SmartDashboard.putNumber("intake CURRENT", intakeMotor.getSupplyCurrent());
        SmartDashboard.putNumber("intake VELOCITY", intakeMotor.getSelectedSensorVelocity());
        SmartDashboard.putBoolean("intake HAS GP", hasGamePiece);
    }

    public void intakeIn() {
        // Set the velocity of the intake motor by percent.
        if (!hasGamePiece) {
            intakeMotor.set(ControlMode.PercentOutput, 1);
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0.1);
        }
    }
    
    public void intakeOut() {
        intakeMotor.set(ControlMode.PercentOutput, -0.5);

        hasGamePiece = false;
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}