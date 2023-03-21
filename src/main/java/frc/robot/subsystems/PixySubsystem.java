// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.Dashboard;

public class PixySubsystem extends SubsystemBase {
    /** Creates a new PixySubsystem. */
    double minPixyVoltage = 1.5;
    double maxPixyVoltage = 2.7;
    double lastKnownPosition = 0;

    AnalogInput pixyX = new AnalogInput(0);

    //AnalogPotentiometer pixyX = new AnalogPotentiometer(0, 2, 0);

    public PixySubsystem() {
        //pixyX.setAverageBits(8);
    }

    public void updateConePosition()    {
        double pos = pixyX.getVoltage();

        if (pos < minPixyVoltage){
            pos = minPixyVoltage;
        } else if (pos > maxPixyVoltage) {
            pos = maxPixyVoltage;
        }

        //Takes the current voltage and subtracts the minimum voltage, multiplies by our range, then divides by our voltage range, subtracting 6 to output the distance from center
        lastKnownPosition = (pos - minPixyVoltage) * 12 / (maxPixyVoltage - minPixyVoltage) - 6;
    }

    public double getLastKnownPosition(){
        return lastKnownPosition;
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        Dashboard.getInstance().putData("Pixy Cam Cone Offset", lastKnownPosition);
        
        Dashboard.getInstance().putData("Pixy Cam Voltage", pixyX.getVoltage());
    }
}
