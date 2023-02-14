// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonSRX intakeMotor = new TalonSRX(Constants.Intake.INTAKE_MOTOR_PWM_PORT);
    private DoubleSolenoid intakesolenoide = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);

    /** Creates a new Intake. */
    public IntakeSubsystem() {
        intakeMotor.configPeakCurrentLimit(6); // don't activate current limit until current exceeds 30 A ...
        intakeMotor.configPeakCurrentDuration(100); // ... for at least 100 ms
        intakeMotor.configContinuousCurrentLimit(5); // once current-limiting is actived, hold at 20A
        intakeMotor.enableCurrentLimit(true); 
    }

    public void intakeIn() {
        intakesolenoide.set(Value.kForward);
        intakeMotor.set(ControlMode.PercentOutput, 0.1);
    }  

    public void intakeOut() {
        intakesolenoide.set(Value.kForward);
        intakeMotor.set(ControlMode.PercentOutput,  -0.1);
    }
    
    public void stop() {
        intakesolenoide.set(Value.kReverse);
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}