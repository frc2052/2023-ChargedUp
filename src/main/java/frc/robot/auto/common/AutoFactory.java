// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.auto.HungryHungryHippoAuto;
import frc.robot.auto.MiddleScoreOneBalanceAuto;
import frc.robot.auto.MiddleScoreOnePickupBalanceAuto;
import frc.robot.auto.ScoreOneBalanceAuto;
import frc.robot.auto.ScoreTwoBalanceAuto;
import frc.robot.auto.ScoreTwoUpperAuto;
import frc.robot.io.Dashboard;

/**
 * Responsible for selecting, compiling, and recompiling autos before the start of a match.
 */
public class AutoFactory {
    private final Supplier<Auto> autoSupplier;
    private final Supplier<AutoConfiguration> autoConfigurationSupplier;

    private final AutoRequirements autoRequirements;

    private Auto currentAuto;
    private AutoConfiguration currentAutoConfiguration;

    private AutoBase compiledAuto;

    public AutoFactory(
        Supplier<Auto> autoSupplier,
        Supplier<AutoConfiguration> autoConfigurationSupplier,
        AutoRequirements autoRequirements
    ) {
        this.autoSupplier = autoSupplier;
        this.autoConfigurationSupplier = autoConfigurationSupplier;

        this.autoRequirements = autoRequirements;
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
        compiledAuto = currentAuto.getInstance(currentAutoConfiguration, autoRequirements);

        if (compiledAuto != null) {
            DashboardAutoRequirements autoRequirements = compiledAuto.getClass().getAnnotation(DashboardAutoRequirements.class);
            Dashboard.getInstance().updateAutoChoosers(autoRequirements != null ? autoRequirements.requirements() : null);

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
        NO_AUTO(null),
        SCORE_ONE_BALANCE(ScoreOneBalanceAuto.class),
        SCORE_TWO_BALANCE(ScoreTwoBalanceAuto.class),
        SCORE_TWO_UPPER(ScoreTwoUpperAuto.class),
        MIDDLE_SCORE_ONE_PICKUP_BALANCE(MiddleScoreOnePickupBalanceAuto.class),
        MIDDLE_SCORE_ONE_NONE_BALANCE(MiddleScoreOneBalanceAuto.class),
        HUNGRY_HUNGRY_HIPPO(HungryHungryHippoAuto.class);

        private final Class<? extends AutoBase> autoClass;

        private Auto(Class<? extends AutoBase> autoClass) {
            this.autoClass = autoClass;
        }

        public AutoBase getInstance(AutoConfiguration autoConfiguration, AutoRequirements autoRequirements) {
            if (autoClass != null) {
                try {
                    return autoClass.getConstructor(AutoConfiguration.class, AutoRequirements.class).newInstance(autoConfiguration, autoRequirements);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            
            return null;
        }
    }

    public static enum Grid implements DashboardAutoRequirement {
        LEFT_GRID,
        MIDDLE_GRID,
        RIGHT_GRID;
    }

    public static enum Node implements DashboardAutoRequirement {
        LEFT_CONE,
        MIDDLE_CUBE,
        RIGHT_CONE;
    }

    public static enum GamePiece implements DashboardAutoRequirement {
        FAR_LEFT_GAME_PIECE,
        MIDDLE_LEFT_GAME_PIECE,
        MIDDLE_RIGHT_GAME_PIECE,
        FAR_RIGHT_GAME_PIECE,
        NO_GAME_PIECE;
    }

    public static enum ChargeStation implements DashboardAutoRequirement {
        NO_BALANCE,
        BALANCE;
    }
}
