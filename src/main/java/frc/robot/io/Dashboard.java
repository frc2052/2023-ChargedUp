// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

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
    // private final SendableChooser<Type> exampleChooser;
    private final SendableChooser<Autos> autoChooser;
    private final SendableChooser<Node> nodeChooser;
    private final SendableChooser<Grid> gridChooser;
    private final SendableChooser<Channel> channelChooser;
    private final SendableChooser<GamePiece> gamePieceSelectable;

    private Dashboard() {
        SmartDashboard.putBoolean(
                Constants.Dashboard.FIELD_RELATIVE_KEY,
                Constants.Dashboard.FIELD_RELATIVE_DEFAULT);

        autoChooser = new SendableChooser<Autos>();
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name, auto);
        }
        autoChooser.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        SmartDashboard.putData("Auto", autoChooser);

        nodeChooser = new SendableChooser<Node>();
        for (Node node : Node.values()) {
            nodeChooser.addOption(node.name(), node);
        }
        nodeChooser.setDefaultOption(Node.values()[0].name(), Node.values()[0]);
        SmartDashboard.putData("Node", nodeChooser);

        gridChooser = new SendableChooser<Grid>();
        for (Grid grid : Grid.values()) {
            gridChooser.addOption(grid.name(), grid);
        }
        gridChooser.setDefaultOption(Grid.values()[0].name(), Grid.values()[0]);
        SmartDashboard.putData("Grid", gridChooser);

        channelChooser = new SendableChooser<Channel>();
        for (Channel channel : Channel.values()) {
            channelChooser.addOption(channel.name(), channel);
        }
        channelChooser.setDefaultOption(Channel.values()[0].name(), Channel.values()[0]);
        SmartDashboard.putData("Channel", channelChooser);
        // for some reason there are two different "putData"s, which is only slightly
        // confusing

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
        SmartDashboard.putString("Node Name", getNode().name());
        SmartDashboard.putString("Grid Name", getGrid().name());
        SmartDashboard.putString("Channel", getChannel().name());
        SmartDashboard.putString("Game Piece", getGamePiece().name());
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

    public Autos getAuto() {
        return autoChooser.getSelected();
    }

    public Node getNode() {
        return nodeChooser.getSelected();
    }

    public Grid getGrid() {
        return gridChooser.getSelected();
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

    public static enum Autos {
        DynamicAutoFactory("DynamicAuto", "Description"),
        EXAMPLE_AUTO("Example", "Description");

        // ForwardAuto("MoveForward", "Moves forward");

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