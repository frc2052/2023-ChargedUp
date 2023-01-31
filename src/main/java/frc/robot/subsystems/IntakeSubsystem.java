// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final VictorSPX intakeMotor;

    /** Creates a new Intake. */
    public IntakeSubsystem() {
        intakeMotor = new VictorSPX(Constants.Intake.INTAKE_MOTOR_PWM);
    }
    
    public void intake() {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, 1.0);
    } 
    
    public void reverseIntake() {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, -1.0);
    }

    public void stop() {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
}