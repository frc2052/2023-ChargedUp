// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MatchEndBlockCommand extends CommandBase {
    /** Creates a new MatchEndCommand. */
    public MatchEndBlockCommand() { }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return DriverStation.getMatchTime() < 0.25;
    }
}
