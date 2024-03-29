// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;

public class PneumaticsSubsystem extends SubsystemBase {
    private final PneumaticHub pneumaticHub;

    public PneumaticsSubsystem() {
        pneumaticHub = new PneumaticHub(Constants.Compressor.PNEUMATIC_HUB_ID);

        // Min and max recharge pressure, pressure maxes out at 115
        pneumaticHub.enableCompressorAnalog(Constants.Compressor.COMPRESSOR_MIN_PRESSURE, Constants.Compressor.COMPRESSOR_MAX_PRESSURE);
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(Constants.Dashboard.PRESSURE_KEY, pneumaticHub.getPressure(0));
    }
}
