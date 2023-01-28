// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.annotation.JsonFormat.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
  
  private VictorSPX intakeMotor = new VictorSPX(Constants.Intake.INTAKE_MOTOR_PWM_PORT);

  public void intake() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.1);
  }  
  public void reverseIntake() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput,  -0.1); }
  
  public void stop() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}