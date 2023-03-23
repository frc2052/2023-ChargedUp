// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import java.util.function.Function;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.AutoConfiguration;

/** Add your docs here. */
// Trying something different this year, instead of a normal class
// the dashboard this year is using a singleton class, so only one instance will
// run at once
public class Dashboard {
    private static Dashboard INSTANCE;

    // Creates sendable choosers
    private final SendableChooser<DriveMode> driveModeChooser;

    private final SendableChooser<Auto> autoChooser;
    private final SendableChooser<Node> startingNodeChooser;
    private final SendableChooser<Grid> startingGridChooser;
    private final SendableChooser<Channel> exitChannelChooser;
    private final SendableChooser<GamePiece> gamePieceChooser;
    private final SendableChooser<Grid> scoreGridChooser;
    private final SendableChooser<Node> scoreNodeChooser;

    private Dashboard() {
        //Creates options for different choosers
        driveModeChooser = new SendableChooser<DriveMode>();
        driveModeChooser.addOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.ROBOT_CENTRIC.name(), DriveMode.ROBOT_CENTRIC);
        driveModeChooser.setDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        SmartDashboard.putData(Constants.Dashboard.DRIVE_MODE_KEY, driveModeChooser);

        autoChooser = new SendableChooser<Auto>();
        for (Auto auto : Auto.values()) {
            autoChooser.addOption(auto.name, auto);
        }
        autoChooser.setDefaultOption(Auto.NO_AUTO.name(), Auto.NO_AUTO);
        SmartDashboard.putData("Auto", autoChooser);

        startingNodeChooser = new SendableChooser<Node>();
        for (Node node : Node.values()) {
            startingNodeChooser.addOption(node.name(), node);
        }
        startingNodeChooser.setDefaultOption(Node.MIDDLE_CUBE.name(), Node.MIDDLE_CUBE);
        SmartDashboard.putData("Starting Node", startingNodeChooser);

        startingGridChooser = new SendableChooser<Grid>();
        for (Grid grid : Grid.values()) {
            startingGridChooser.addOption(grid.name(), grid);
        }
        startingGridChooser.setDefaultOption(Grid.values()[0].name(), Grid.values()[0]);
        SmartDashboard.putData("Starting Grid", startingGridChooser);

        exitChannelChooser = new SendableChooser<Channel>();
        for (Channel channel : Channel.values()) {
            exitChannelChooser.addOption(channel.name(), channel);
        }
        exitChannelChooser.setDefaultOption(Channel.values()[0].name(), Channel.values()[0]);
        SmartDashboard.putData("Exit Channel", exitChannelChooser);

        gamePieceChooser = new SendableChooser<GamePiece>();
        for (GamePiece gamePiece : GamePiece.values()) {
            gamePieceChooser.addOption(gamePiece.name(), gamePiece);
        }
        gamePieceChooser.setDefaultOption(GamePiece.values()[0].name(), GamePiece.values()[0]);
        SmartDashboard.putData("Game Piece", gamePieceChooser);
   
        SmartDashboard.putBoolean("Score Game Piece", true);

        scoreNodeChooser = new SendableChooser<Node>();
        for (Node node : Node.values()) {
            scoreNodeChooser.addOption(node.name(), node);
        }
        scoreNodeChooser.setDefaultOption(Node.values()[0].name(), Node.values()[0]);
        SmartDashboard.putData("Score Node", scoreNodeChooser);

        scoreGridChooser = new SendableChooser<Grid>();
        for (Grid grid : Grid.values()) {
            scoreGridChooser.addOption(grid.name(), grid);
        }
        scoreGridChooser.setDefaultOption(Grid.values()[0].name(), Grid.values()[0]);
        SmartDashboard.putData("Score Grid", scoreGridChooser);

        SmartDashboard.putBoolean("End Charge Station", true);

        SmartDashboard.putNumber("Score Offset Degrees", 0);
    }

    // updates dashboard with needed information
    public void updateDashboard() {
        SmartDashboard.putString("Auto Description", getAuto().description);
    }

    public <V> void putData(String key, V value) {
        if (value instanceof Float) {
            SmartDashboard.putNumber(key, (Float) value);
        } else if (value instanceof Integer) {
            SmartDashboard.putNumber(key, (Integer) value);
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

    public double getScoreOffsetDegrees() {
        return SmartDashboard.getNumber("Score Offset Degrees", 0);
    }

    public DriveMode getDriveMode() {
        return driveModeChooser.getSelected();      
    }

    public Auto getAuto() {
        return autoChooser.getSelected();
    }

    public Node getStartingNode() {
        return startingNodeChooser.getSelected();
    }

    public Grid getStartingGrid() {
        return startingGridChooser.getSelected();
    }

    public Channel getExitChannel() {
        return exitChannelChooser.getSelected();
    }

    public GamePiece getGamePiece() {
        return gamePieceChooser.getSelected();
    }

    public boolean scoreGamePiece() {
        return SmartDashboard.getBoolean("Score Game Piece", false);
    }

    public Node getScoreNode() {
        return scoreNodeChooser.getSelected();
    }

    public Grid getScoreGrid() {
        return scoreGridChooser.getSelected();
    }

    public boolean endChargeStation(){
        return SmartDashboard.getBoolean("End Charge Station", true);
    }

    public AutoConfiguration getAutoConfiguration() {
        return new AutoConfiguration(
            getStartingGrid(), 
            getStartingNode(), 
            getExitChannel(), 
            getGamePiece(), 
            scoreGamePiece(), 
            getScoreGrid(), 
            getScoreNode(), 
            endChargeStation()
        );
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

    public static enum Auto {
        NO_AUTO("NO AUTO", "NO AUTO"),
        //DYNAMIC_AUTO_FACTORY("DynamicAutoFactory", "Description"),
        //TEST_LEFT_SCORE_ONE_BALANCE("Test Score One Balance", ""),
        SCORE_ONE_BALANCE("Score One Balance", "Description"),
        SCORE_TWO_BALANCE("Score Two Balance", "Description"),
        MIDDLE_SCORE_ONE_EXIT("Middle Score One Exit", "Description"),
        MIDDLE_SCORE_ONE_BALANCE("Middle Score One Balance", "Description");

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
