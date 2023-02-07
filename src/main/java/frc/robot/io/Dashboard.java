// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.io.Dashboard.Autos.ChargingBalance;
import frc.robot.io.Dashboard.Autos.DriveMode;

/** Add your docs here. */
/*This Dashboard is a singleton class, meaning it only creates one instance at a time, that can be accessed
globally*/
public class Dashboard {
    private static Dashboard INSTANCE;

    // Creates sendable choosers
    // private final SendableChooser<Type> exampleChooser;
    // Auto and Charge Station Balancing Chooser
    private final SendableChooser<Autos> autoChooser;
    private final SendableChooser<ChargingBalance> balancingChooser;
    private final SendableChooser<DriveMode> driveModeSelect;
    

    private Dashboard() {
        SmartDashboard.putBoolean(
            Constants.Dashboard.FIELD_RELATIVE_KEY,
            Constants.Dashboard.FIELD_RELATIVE_DEFAULT
        );


        //Creates options for different choosers
        autoChooser = new SendableChooser<Autos>();
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name, auto);
        }
        autoChooser.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        SmartDashboard.putData("Auto", autoChooser);

        balancingChooser = new SendableChooser<ChargingBalance>();
        for (ChargingBalance chargingBalance : ChargingBalance.values()){
            balancingChooser.addOption(chargingBalance.name(), chargingBalance);
        }
        balancingChooser.setDefaultOption(ChargingBalance.NOT_BALANCED.name(), ChargingBalance.values()[1]);
        SmartDashboard.putData("Charging Balancer", balancingChooser);

        driveModeSelect = new SendableChooser<DriveMode>();
        for (DriveMode driveMode : DriveMode.values()){
            driveModeSelect.addOption(driveMode.name(), driveMode);
        }
        driveModeSelect.setDefaultOption(DriveMode.ROBOT_CENTRIC.name(), getDriveMode());
        SmartDashboard.putData("Drive Mode", driveModeSelect);

    }
// updates dashboard with needed information
    public void updateDashboard() {
       SmartDashboard.putString("Auto Description", getAuto().description);
       SmartDashboard.putString("Charging Station Balancer", getChargingBalance().name());
    }


    public boolean isFieldRelative() {
        return SmartDashboard.getBoolean(
            Constants.Dashboard.FIELD_RELATIVE_KEY,
            Constants.Dashboard.FIELD_RELATIVE_DEFAULT
        );
    }

    // creates getSelected command
    public Autos getAuto() {
        return autoChooser.getSelected();
    }
    public ChargingBalance getChargingBalance(){
        return balancingChooser.getSelected();
    }
    public DriveMode getDriveMode(){
        return driveModeSelect.getSelected();
    }

    // Creates new dashboard instance
    public static Dashboard getInstance(){
        if (INSTANCE == null) {
           INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }

    // Create enums for Dashboard elements/parts here
    public static enum Autos {
        EXAMPLE_AUTO("Example", "Description");
        //ForwardAuto("MoveForward", "Moves forward");

        private final String name;
        private final String description;

        private Autos(String name, String description) {
            this.name = name;
            this.description = description;
        }

        public static enum ChargingBalance {
            BALANCED,
            NOT_BALANCED,
            }
        
        public enum DriveMode {
                FIELD_CENTRIC,
                ROBOT_CENTRIC,
            }

        public String getName() {
            return name;
        }

        public String getDescription() {
            return description;
        }
    }
}
