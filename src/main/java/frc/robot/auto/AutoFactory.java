// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.function.Supplier;

import frc.robot.io.Dashboard;
import frc.robot.io.Dashboard.Auto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Add your docs here. */
public class AutoFactory {
    private final Supplier<Auto> autoSupplier;
    private final Supplier<AutoConfiguration> autoConfigurationSupplier;

    private final DrivetrainSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final VisionSubsystem vision;

    private Auto currentAuto;
    private AutoConfiguration currentAutoConfiguration;

    private AutoBase compiledAuto;

    public AutoFactory(
        Supplier<Auto> autoSupplier,
        Supplier<AutoConfiguration> autoConfigurationSupplier,
        DrivetrainSubsystem drivetrain, 
        ElevatorSubsystem elevator, 
        IntakeSubsystem intake, 
        ArmSubsystem arm,
        VisionSubsystem vision
    ) {
        this.autoSupplier = autoSupplier;
        this.autoConfigurationSupplier = autoConfigurationSupplier;

        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;
        this.vision = vision;
    }

    public void precompileAuto() {
        if (autoSupplier.get() == currentAuto && autoConfigurationSupplier.get().equals(currentAutoConfiguration)) {
            return;
        }

        Dashboard.getInstance().putData("Auto Compiled", false);

        currentAuto = autoSupplier.get();
        currentAutoConfiguration = autoConfigurationSupplier.get();

        switch (currentAuto) {
            case SCORE_ONE_BALANCE:
                compiledAuto = new RedScoreOneBalanceAuto(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );
                break;

            case SCORE_TWO_BALANCE:
                compiledAuto =  new RedLeftScoreTwoBalanceAuto(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );
                break;

            case MIDDLE_SCORE_ONE_EXIT:
                compiledAuto =  new MiddleScoreOneExitBalance(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );
                break;

            case MIDDLE_SCORE_ONE_BALANCE:
                compiledAuto =  new RedMiddleScoreOneBalance(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );
                break;

            case NO_AUTO:
            default:
                compiledAuto = null;
                break;
        }

        Dashboard.getInstance().putData("Auto Compiled", true);
    }

    public AutoBase getCompiledAuto() {
        return compiledAuto;
    }
}