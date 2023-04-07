// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakePixySubsystem;
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
    private final IntakePixySubsystem intakePixy;
    private final ForwardPixySubsystem forwardPixy;

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
        VisionSubsystem vision,
        IntakePixySubsystem intakePixy,
        ForwardPixySubsystem forwardPixy
    ) {
        this.autoSupplier = autoSupplier;
        this.autoConfigurationSupplier = autoConfigurationSupplier;

        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;
        this.vision = vision;
        this.intakePixy = intakePixy;
        this.forwardPixy = forwardPixy;
    }

    public boolean recompileNeeded() {
        return autoSupplier.get() != currentAuto || !autoConfigurationSupplier.get().equals(currentAutoConfiguration);
    }

    public void recompile() {
        Dashboard.getInstance().putData(Constants.Dashboard.AUTO_COMPILED_KEY, false);

        currentAuto = autoSupplier.get();
        if (currentAuto == null) {
            currentAuto = Auto.NO_AUTO;
        }

        currentAutoConfiguration = autoConfigurationSupplier.get();

        switch (currentAuto) {
            case SCORE_ONE_BALANCE:
                compiledAuto = new ScoreOneBalanceAuto(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm,
                    forwardPixy
                );
                break;

            case SCORE_TWO_BALANCE:
                compiledAuto =  new ScoreTwoBalanceAuto(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm,
                    forwardPixy
                );
                break;

            case SCORE_TWO_UPPER:
                compiledAuto = new ScoreTwoUpperAuto(
                    currentAutoConfiguration, 
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm, 
                    vision, 
                    intakePixy,
                    forwardPixy
                );
                break;

            case MIDDLE_SCORE_ONE_BALANCE:
                compiledAuto =  new MiddleScoreOneBalance(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );
                break;

            case HUNGRY_HUNGRY_HIPPO:
                compiledAuto = new HungryHungryHippoAuto(
                    currentAutoConfiguration, 
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm,
                    forwardPixy
                );
                break;

            case NO_AUTO:
            default:
                compiledAuto = null;
                break;
        }

        if (compiledAuto != null) {
            compiledAuto.init();

            AutoDescription autoDescription = compiledAuto.getClass().getAnnotation(AutoDescription.class);
            if (autoDescription != null) {
                Dashboard.getInstance().putData(Constants.Dashboard.AUTO_DESCRIPTION_KEY, autoDescription.description());
            }
        }

        Dashboard.getInstance().putData(Constants.Dashboard.AUTO_COMPILED_KEY, true);
    }

    public AutoBase getCompiledAuto() {
        return compiledAuto;
    }

    public static enum Auto {
        NO_AUTO,
        SCORE_ONE_BALANCE,
        SCORE_TWO_BALANCE,
        SCORE_TWO_UPPER,
        MIDDLE_SCORE_ONE_BALANCE,
        HUNGRY_HUNGRY_HIPPO;
    }

    public static enum Grid {
        LEFT_GRID,
        MIDDLE_GRID,
        RIGHT_GRID;
    }

    public static enum Node {
        LEFT_CONE,
        MIDDLE_CUBE,
        RIGHT_CONE;
    }

    public static enum Row {
        HYBRID,
        MIDDLE,
        HIGH
    }

    // Path around the charge station either on the left or right side
    public static enum Channel {
        LEFT_CHANNEL,
        RIGHT_CHANNEL;
    }

    public static enum GamePiece {
        FAR_LEFT_GAME_PIECE,
        MIDDLE_LEFT_GAME_PIECE,
        MIDDLE_RIGHT_GAME_PIECE,
        FAR_RIGHT_GAME_PIECE,
        NO_GAME_PIECE;
    }
}
