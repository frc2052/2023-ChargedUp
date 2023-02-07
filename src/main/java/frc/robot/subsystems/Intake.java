// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
  private DoubleSolenoid intakesolenoide = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);

  private VictorSPX intakeMotor = new VictorSPX(Constants.Intake.INTAKE_MOTOR_PWM_PORT);

  public void intake() {
    intakesolenoide.set(Value.kForward);
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.1);
  }  
  public void reverseIntake() {
    intakesolenoide.set(Value.kReverse);
    intakeMotor.set(VictorSPXControlMode.PercentOutput,  -0.1); }
    
  public void stop() {
    intakesolenoide.set(Value.kReverse);
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    

  

  }
  
}