// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.DynamicAutoFactory;

/** Add your docs here. */
// Trying something different this year, instead of a normal class
// the dashboard this year is using a singleton class, so only one instance will
// run at once
public class Dashboard {
    private static Dashboard INSTANCE;

    // Creates sendable choosers
    private final SendableChooser<DriveMode> driveModeChooser;

    private final SendableChooser<Autos> autoChooser;
    private final SendableChooser<Channel> channelChooser;
    private final SendableChooser<GamePiece> gamePieceSelectable;

    private Dashboard() {
        SmartDashboard.putBoolean(
            Constants.Dashboard.FIELD_RELATIVE_KEY,
            Constants.Dashboard.FIELD_RELATIVE_DEFAULT
        );

        //Creates options for different choosers
        driveModeChooser = new SendableChooser<DriveMode>();
        for (DriveMode driveMode : DriveMode.values()){
            driveModeChooser.addOption(driveMode.name(), driveMode);
        }
        driveModeChooser.setDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        SmartDashboard.putData("Drive Mode", driveModeChooser);

        autoChooser = new SendableChooser<Autos>();
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name, auto);
        }
        autoChooser.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        SmartDashboard.putData("Auto", autoChooser);

        channelChooser = new SendableChooser<Channel>();
        for (Channel channel : Channel.values()) {
            channelChooser.addOption(channel.name(), channel);
        }
        channelChooser.setDefaultOption(Channel.values()[0].name(), Channel.values()[0]);
        SmartDashboard.putData("Channel", channelChooser);
        
        gamePieceSelectable = new SendableChooser<GamePiece>();
        for (GamePiece gamePiece : GamePiece.values()) {
            gamePieceSelectable.addOption(gamePiece.name(), gamePiece);
        }
        gamePieceSelectable.setDefaultOption(GamePiece.values()[0].name(), GamePiece.values()[0]);
        SmartDashboard.putData("Game Piece", gamePieceSelectable);
    }

    // updates dashboard with needed information
    public void updateDashboard() {
       SmartDashboard.putString("Auto Description", getAuto().description);
       SmartDashboard.putString("Charging Station Balancer", getChargingBalance().name());
    }

    public <V> void putData(String key, V value) {
        if (value instanceof Number) {
            SmartDashboard.putNumber(key, (Double) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        }
    }

    public boolean isFieldRelative() {
        return SmartDashboard.getBoolean(
                Constants.Dashboard.FIELD_RELATIVE_KEY,
                Constants.Dashboard.FIELD_RELATIVE_DEFAULT);
    }

    // creates getSelected command
    public Autos getAuto() {
        return autoChooser.getSelected();
    }

    public Channel getChannel() {
        return channelChooser.getSelected();
    }

    public GamePiece getGamePiece() {
        return gamePieceSelectable.getSelected();
    }

    public static Dashboard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }

    // Create enums for Dashboard elements/parts here
    public static enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }

    public static enum Autos {
        EXAMPLE_AUTO("Example", "Description"),
        DynamicAutoFactory("DynamicAutoFactory", "Description");
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
}
