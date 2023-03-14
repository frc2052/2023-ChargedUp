// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.Dashboard;

public class PixySubsystem extends SubsystemBase {
    /** Creates a new PixySubsystem. */

    //AnalogInput pixyX = new AnalogInput(0);

    AnalogPotentiometer pixyX = new AnalogPotentiometer(0, 2, 0);

    public PixySubsystem() {
        //pixyX.setAverageBits(8);
    }


    public double getXPct() {

        double xVal = pixyX.get();    

        return xVal;
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run


        Dashboard.getInstance().putData("Pixy Cam Voltage", getXPct());
        
        // Dashboard.getInstance().putData("Pixy Cam Average Voltage", pixyX.getAverageVoltage());
    }
}
