// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;

public class IntakePixySubsystem extends SubsystemBase {
    private final double minPixyVoltage = 1.5;
    private final double maxPixyVoltage = 2.7;
    
    private double lastKnownPosition = 0;

    private final AnalogInput pixyX;

    public IntakePixySubsystem() {
        pixyX = new AnalogInput(0);
    }

    public void updateConePosition()    {
        double pos = pixyX.getVoltage();

        if (pos < minPixyVoltage){
            pos = minPixyVoltage;
        } else if (pos > maxPixyVoltage) {
            pos = maxPixyVoltage;
        }

        // Takes the current voltage and subtracts the minimum voltage, multiplies by our range, 
        // then divides by our voltage range, subtracting 6 to output the distance from center.
        lastKnownPosition = (pos - minPixyVoltage) * 12 / (maxPixyVoltage - minPixyVoltage) - 6;
    }

    public double getLastKnownPositionInches(){
        return lastKnownPosition;
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(Constants.Dashboard.CONE_OFFSET_KEY, lastKnownPosition);
    }
}
