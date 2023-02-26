// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
// Trying something different this year, instead of a normal class
// the dashboard this year is using a singleton class, so only one instance will
// run at once
public class Dashboard {
    private static Dashboard INSTANCE;

    // Creates sendable choosers
    private final SendableChooser<DriveMode> driveModeChooser;

    private final SendableChooser<Autos> autoChooser;
    private final SendableChooser<Node> nodeChooser;
    private final SendableChooser<Grid> gridChooser;
    private final SendableChooser<Channel> exitChannelChooser;
    private final SendableChooser<GamePiece> gamePieceChooser;
    private final SendableChooser<Grid> scoreGridChooser;
    private final SendableChooser<Node> scoreNodeChooser;
    private final SendableChooser<Channel> enterChannelChooser;

    private Dashboard() {
        //Creates options for different choosers
        driveModeChooser = new SendableChooser<DriveMode>();
        driveModeChooser.addOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.ROBOT_CENTRIC.name(), DriveMode.ROBOT_CENTRIC);
        driveModeChooser.setDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        SmartDashboard.putData(Constants.Dashboard.DRIVE_MODE_KEY, driveModeChooser);

        autoChooser = new SendableChooser<Autos>();
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name, auto);
        }
        autoChooser.setDefaultOption(Autos.NO_AUTO.name, Autos.NO_AUTO);
        SmartDashboard.putData("Auto", autoChooser);

        nodeChooser = new SendableChooser<Node>();
        for (Node node : Node.values()) {
            nodeChooser.addOption(node.name(), node);
        }
        nodeChooser.setDefaultOption(Node.values()[0].name(), Node.values()[0]);
        SmartDashboard.putData("DA: Node", nodeChooser);

        gridChooser = new SendableChooser<Grid>();
        for (Grid grid : Grid.values()) {
            gridChooser.addOption(grid.name(), grid);
        }
        gridChooser.setDefaultOption(Grid.values()[0].name(), Grid.values()[0]);
        SmartDashboard.putData("DA: Grid", gridChooser);

        exitChannelChooser = new SendableChooser<Channel>();
        for (Channel channel : Channel.values()) {
            exitChannelChooser.addOption(channel.name(), channel);
        }
        exitChannelChooser.setDefaultOption(Channel.values()[0].name(), Channel.values()[0]);
        SmartDashboard.putData("DA: Channel", exitChannelChooser);

        gamePieceChooser = new SendableChooser<GamePiece>();
        for (GamePiece gamePiece : GamePiece.values()) {
            gamePieceChooser.addOption(gamePiece.name(), gamePiece);
        }
        gamePieceChooser.setDefaultOption(GamePiece.values()[0].name(), GamePiece.values()[0]);
        SmartDashboard.putData("DA: Game Piece", gamePieceChooser);
   
        scoreNodeChooser = new SendableChooser<Node>();
        for (Node node : Node.values()) {
            scoreNodeChooser.addOption(node.name(), node);
        }
        scoreNodeChooser.setDefaultOption(Node.values()[0].name(), Node.values()[0]);
        SmartDashboard.putData("DA: Score Node", scoreNodeChooser);

        scoreGridChooser = new SendableChooser<Grid>();
        for (Grid grid : Grid.values()) {
            scoreGridChooser.addOption(grid.name(), grid);
        }
        scoreGridChooser.setDefaultOption(Grid.values()[0].name(), Grid.values()[0]);
        SmartDashboard.putData("DA: Score Grid", scoreGridChooser);

        enterChannelChooser = new SendableChooser<Channel>();
        for (Channel channel : Channel.values()) {
            enterChannelChooser.addOption(channel.name(), channel);
        }
        enterChannelChooser.setDefaultOption(Channel.values()[0].name(), Channel.values()[0]);
        SmartDashboard.putData("DA: Enter Channel", enterChannelChooser);

        SmartDashboard.putBoolean("DA: End Charge Station", true);
    }

    // updates dashboard with needed information
    public void updateDashboard() {
        // SmartDashboard.putString("Auto Description", getAuto().description);
        SmartDashboard.putBoolean("DA: Score Game Piece", false);
    }

    public boolean getScoreGamePiece() {
       return SmartDashboard.getBoolean("DA: Score Game Piece", false);
    }

    public boolean endChargeStation(){
        return SmartDashboard.getBoolean("DA: End Charge Station", true);
    }

    public <V> void putData(String key, V value) {
        if (value instanceof Float) {
            SmartDashboard.putNumber(key, (Float) value);
        } else if (value instanceof Number) {
            SmartDashboard.putNumber(key, (Double) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else if (value instanceof Sendable) {
            Shuffleboard.getTab("main").add(key, (Sendable) value);
        }
    }

    public DriveMode getDriveMode() {
        return driveModeChooser.getSelected();      
    }

    public Autos getAuto() {
        return autoChooser.getSelected();
    }

    public Node getNode() {
        return nodeChooser.getSelected();
    }

    public Grid getGrid() {
        return gridChooser.getSelected();
    }

    public Channel getExitChannel() {
        return exitChannelChooser.getSelected();
    }

    public GamePiece getGamePiece() {
        return gamePieceChooser.getSelected();
    }

    public Node getScoreNode() {
        return scoreNodeChooser.getSelected();
    }

    public Grid getScoreGrid() {
        return scoreGridChooser.getSelected();
    }

    public Channel getEnterChannel() {
        return enterChannelChooser.getSelected();
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
        NO_AUTO("NO AUTO", "Description"),
        DYNAMIC_AUTO_FACTORY("DynamicAutoFactory", "Description"),
        RED_LEFT_SCORE_ONE_BALANCE("Red Left Score One Balance", "Description"),
        RED_LEFT_SCORE_TWO_BALANCE("Red Left Score Two Balance", "Description"),
        MIDDLE_SCORE_ONE_BALANCE("Middle Score One Bwe=alance", "Description");


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
