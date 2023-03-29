// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmInCommand extends CommandBase {
    private final ArmSubsystem arm;

    public ArmInCommand(ArmSubsystem arm) {
        this.arm = arm;

        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        arm.armIn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
