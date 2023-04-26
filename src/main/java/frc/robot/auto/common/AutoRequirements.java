// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakePixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Add your docs here. */
public class AutoRequirements {
    private final DrivetrainSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final VisionSubsystem vision;
    private final IntakePixySubsystem intakePixy;
    private final ForwardPixySubsystem forwardPixy;

    public AutoRequirements(
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator,
        IntakeSubsystem intake, 
        ArmSubsystem arm, 
        VisionSubsystem vision,
        IntakePixySubsystem intakePixy,
        ForwardPixySubsystem forwardPixy
    ) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;
        this.vision = vision;
        this.intakePixy = intakePixy;
        this.forwardPixy = forwardPixy;
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public ElevatorSubsystem getElevator() {
        return elevator;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    public ArmSubsystem getArm() {
        return arm;
    }

    public VisionSubsystem getVision() {
        return vision;
    }

    public IntakePixySubsystem getIntakePixy() {
        return intakePixy;
    }

    public ForwardPixySubsystem getForwardPixy() {
        return forwardPixy;
    }
}
