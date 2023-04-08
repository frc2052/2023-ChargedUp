// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class ScoreThree extends ScorePickUpAutoBase {
    public ScoreThree(
        AutoConfiguration autoConfiguration, 
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator,
        IntakeSubsystem intake, 
        ArmSubsystem arm, 
        ForwardPixySubsystem forwardPixy
    ) {
        super(autoConfiguration, drivetrain, elevator, intake, arm, forwardPixy);
    }
}
