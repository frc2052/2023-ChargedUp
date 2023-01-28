// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
// Trying something different this year, instead of a normal class
// the dashboard this year is using a singleton class, so only one instance will run at once
public class Dashboard {
    private static Dashboard INSTANCE;

    private final SendableChooser<Autos> autoChooser;

    private Dashboard() {
        SmartDashboard.putBoolean(
            Constants.Dashboard.FIELD_RELATIVE_KEY,
            Constants.Dashboard.FIELD_RELATIVE_DEFAULT
        );

        autoChooser = new SendableChooser<Autos>();
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name, auto);
        }
        autoChooser.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        SmartDashboard.putData("Auto", autoChooser);
    }

    public void updateDashboard() {
        SmartDashboard.putString("Auto Description", getAuto().description);
        SmartDashboard.putString("Node Description", getNode().description);
    }
 /*UwU, senpai, wait for me kitty >//.//< */
    public boolean isFieldRelative() {
        return SmartDashboard.getBoolean(
            Constants.Dashboard.FIELD_RELATIVE_KEY,
            Constants.Dashboard.FIELD_RELATIVE_DEFAULT
        );
    }

    public Autos getAuto() {
        return autoChooser.getSelected();
    }
    public Autos getNode() {
        return autoChooser.getSelected();
    }

    public static Dashboard getInstance(){
        if (INSTANCE == null) {
           INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }

    public static enum Autos {
        EXAMPLE_AUTO("Example", "Description");
        //ForwardAuto("MoveForward", "Moves forward");

        private final String name;
        private final String description;

        private Autos(String name, String description) {
            this.name = name;
            this.description = description;
        }

     public static enum Grid {
            LEFT_GRID,
            MIDDLE_CONE,
            RIGHT_CONE,
            }
            
        public static enum Node {
            LEFT_CONE,
            MIDDLE_CONE,
            RIGHT_CUBE,
            }
            
        public static enum Channel {
            LEFT_CHANNEL,
            RIGHT_CHANNEL,
            }
            
        public static enum GamePiece {
            FAR_LEFT_GAME_PIECE,
            MIDDLE_LEFT_GAME_PIECE,
            FAR_RIGHT_GAME_PIECE,
            NO_GAME_PIECE,
            }
  
        public String getName() {
            return name;
        }

        public String getDescription() {
            return description;
        }
    }
}
/*"All hail Matt, the Rizz King"*/