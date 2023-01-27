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
public class DashboardSingleton {
    private static DashboardSingleton INSTANCE;
    private final SendableChooser<Autos> autoChooser;
    
    public static DashboardSingleton getInstance(){
    
        if (INSTANCE == null)
           INSTANCE = new DashboardSingleton();
        
        return INSTANCE;
    }

    private DashboardSingleton() {
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


    public static enum Autos {
        EXAMPLE_AUTO("Example", "Description");
        //ForwardAuto("MoveForward", "Moves forward");

        private final String name;
        private final String description;

        private Autos(String name, String description) {
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
}