// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PixySubsystem extends SubsystemBase {
  /** Creates a new PixySubsystem. */

  AnalogInput pixyX = new AnalogInput(0);
  public PixySubsystem() {}


  public double getXPct() {

    double xVal = pixyX.getVoltage();    

    return xVal;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    System.out.println("******************************** PIXY  " + getXPct());

  }


}
