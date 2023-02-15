// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private final DoubleSolenoid intakesolenoid;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        intakesolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            Constants.Arm.ARM_SOLENOID_FORWARD_CHANNEL, 
            Constants.Arm.ARM_SOLENOID_REVERSE_CHANNEL
        );
    }

    public void armIn() {
        intakesolenoid.set(Value.kReverse);
    }

    public void armOut() {
        intakesolenoid.set(Value.kForward);
    }

    public void toggleArm() {
        intakesolenoid.toggle();
    }
}
