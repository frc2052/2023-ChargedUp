// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor;

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
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(Constants.Dashboard.INTAKE_CURRENT_KEY, intakeMotor.getSupplyCurrent());
        if (intakeMotor.getSupplyCurrent()  < Constants.Intake.INTAKE_CRUISE_CURRENT_AMPS + 1 && intakeMotor.getSupplyCurrent() > 1){
            //reusing status mode, just because we want white
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CURRENT_LIMITING);
        } else if (LEDSubsystem.getInstance().getLEDStatusMode() == LEDStatusMode.NO_AUTO && !LEDSubsystem.getInstance().getRobotDisabled()){
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.OFF);
        }
    }

    public void intakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_IN_SPEED);
        
        System.out.println("Intaking in!!!");
    }
    
    public void slowIntakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_IN_SLOW_SPEED);
    }

    public void intakeOut() {
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_OUT_SPEED);
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }
}