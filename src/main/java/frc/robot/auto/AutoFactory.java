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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PixySubsystem;
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
    private final PixySubsystem pixy;

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
        PixySubsystem pixy
    ) {
        this.autoSupplier = autoSupplier;
        this.autoConfigurationSupplier = autoConfigurationSupplier;

        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.intake = intake;
        this.arm = arm;
        this.vision = vision;
        this.pixy = pixy;
    }

    public void precompileAuto() {
        if (autoSupplier.get() == currentAuto && autoConfigurationSupplier.get().equals(currentAutoConfiguration)) {
            return;
        }

        forceRecompile();
    }

    public void forceRecompile() {
        Dashboard.getInstance().putData(Constants.Dashboard.AUTO_COMPILED_KEY, false);

        currentAuto = autoSupplier.get();
        currentAutoConfiguration = autoConfigurationSupplier.get();

        switch (currentAuto) {
            case SCORE_ONE_BALANCE:
                compiledAuto = new ScoreOneBalanceAuto(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );
                break;

            case SCORE_TWO_BALANCE:
                compiledAuto =  new ScoreTwoBalanceAuto(
                    currentAutoConfiguration,
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
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
                    pixy
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
                    arm
                );
                break;

            case NO_AUTO:
            default:
                compiledAuto = null;
                break;
        }

        if (compiledAuto != null) {
            compiledAuto.init();

            AutoDescription description = compiledAuto.getClass().getAnnotation(AutoDescription.class);
            if (description != null) {
                System.out.println(description.description());
            }
        }

        Dashboard.getInstance().putData(Constants.Dashboard.AUTO_COMPILED_KEY, true);
    }

    public AutoBase getCompiledAuto() {
        return compiledAuto;
    }

    public static enum Auto {
        NO_AUTO("NO AUTO", "NO AUTO"),
        SCORE_ONE_BALANCE("Score One Balance", "Description"),
        SCORE_TWO_BALANCE("Score Two Balance", "Description"),
        SCORE_TWO_UPPER("Score Two Upper", "Description"),
        MIDDLE_SCORE_ONE_BALANCE("Middle Score One Balance", "Description"),
        HUNGRY_HUNGRY_HIPPO("Hungry Hungry Hippo", "Nom nom");

        private final String name;
        private final String description;

        private Auto(String name, String description) {
            this.name = name;
            this.description = description;
        }
  
        public String getName() {
            return name;
        }

        public String getDescription() {
            return description;
        }
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
