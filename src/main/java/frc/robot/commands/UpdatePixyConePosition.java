// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class UpdatePixyConePosition extends CommandBase {
  /** Creates a new updatePixyConePosition. */
    private PixySubsystem pixyCam;

  public UpdatePixyConePosition(
    PixySubsystem pixyCam
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pixyCam = pixyCam;
  }

  @Override
  public void execute() {
    pixyCam.updateConePosition();
    LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
