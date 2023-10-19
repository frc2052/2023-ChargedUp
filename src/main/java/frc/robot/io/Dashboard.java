// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.DashboardAutoRequirement;
import frc.robot.auto.common.AutoFactory.Auto;
import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.GamePiece;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;

// Trying something different this year, instead of a normal class
// the dashboard this year is using a singleton class, so only one instance will
// run at once
public class Dashboard {
    private static Dashboard INSTANCE;
    private final NetworkTableInstance ntinst;
    private final NetworkTable rPiTable;

    // Creates sendable choosers

    private final SendableChooser<Integer> tagGoalChooser;

    private final SendableChooser<DriveMode> driveModeChooser;

    private final SendableChooser<Auto> autoChooser;

    private final SendableChooser<Node> startingNodeChooser;
    private final SendableChooser<Grid> startingGridChooser;
    private final SendableChooser<GamePiece> gamePieceChooser;
    private final SendableChooser<ChargeStation> chargeStationChooser;

    private Dashboard() {

        ntinst = NetworkTableInstance.getDefault();
        rPiTable = ntinst.getTable("RaspberryPi");

        //Creates options for different choosers

        tagGoalChooser = new SendableChooser<Integer>();
        for (int i = 0; i < 4; i++){
            tagGoalChooser.addOption("Tag " + i, i);

        }
        tagGoalChooser.setDefaultOption("Tag 0", 0);

        driveModeChooser = new SendableChooser<DriveMode>();
        driveModeChooser.addOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.ROBOT_CENTRIC.name(), DriveMode.ROBOT_CENTRIC);
        driveModeChooser.setDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        SmartDashboard.putData(Constants.Dashboard.DRIVE_MODE_KEY, driveModeChooser);

        autoChooser = new SendableChooser<Auto>();
        for (Auto auto : Auto.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
        autoChooser.setDefaultOption(Auto.NO_AUTO.name(), Auto.NO_AUTO);
        SmartDashboard.putData("Auto", autoChooser);

        startingNodeChooser = new SendableChooser<Node>();
        for (Node node : Node.values()) {
            startingNodeChooser.addOption(node.name(), node);
        }
        startingNodeChooser.setDefaultOption(Node.MIDDLE_CUBE.name(), Node.MIDDLE_CUBE);

        startingGridChooser = new SendableChooser<Grid>();
        for (Grid grid : Grid.values()) {
            startingGridChooser.addOption(grid.name(), grid);
        }
        startingGridChooser.setDefaultOption(Grid.LEFT_GRID.name(), Grid.LEFT_GRID);

        gamePieceChooser = new SendableChooser<GamePiece>();
        for (GamePiece gamePiece : GamePiece.values()) {
            gamePieceChooser.addOption(gamePiece.name(), gamePiece);
        }
        gamePieceChooser.setDefaultOption(GamePiece.NO_GAME_PIECE.name(), GamePiece.NO_GAME_PIECE);

        chargeStationChooser = new SendableChooser<ChargeStation>();
        for (GamePiece gamePiece : GamePiece.values()) {
            gamePieceChooser.addOption(gamePiece.name(), gamePiece);
        }
        chargeStationChooser.setDefaultOption(ChargeStation.BALANCE.name(), ChargeStation.BALANCE);

        SmartDashboard.putBoolean("Pixy Cam Broken", false);

        updateAutoChoosers(null);
    }

    public void updateAutoChoosers(Class<? extends DashboardAutoRequirement>[] autoRequirements) {
        if (autoRequirements != null) {
            // SmartDashboard.putData("Starting Grid", null);
            // SmartDashboard.putData("Starting Node", null);
            // SmartDashboard.putData("Game Piece", null);
            // SmartDashboard.putData("Charge Station", null);

            for (Class<? extends DashboardAutoRequirement> autoRequirement : autoRequirements) {
                if (autoRequirement.equals(Grid.class)) {
                    SmartDashboard.putData("Starting Grid", startingGridChooser);
                } else if (autoRequirement.equals(Node.class)) {
                    SmartDashboard.putData("Starting Node", startingNodeChooser);
                } else if (autoRequirement.equals(GamePiece.class)) {
                    SmartDashboard.putData("Game Piece", gamePieceChooser);
                } else if (autoRequirement.equals(ChargeStation.class)) {
                    SmartDashboard.putData("Charge Station", chargeStationChooser);
                }
            }
        } else {
            SmartDashboard.putData("Starting Grid", startingGridChooser);
            SmartDashboard.putData("Starting Node", startingNodeChooser);
            SmartDashboard.putData("Game Piece", gamePieceChooser);
            SmartDashboard.putData("Charge Station", chargeStationChooser);
        }
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

    public int getGoalTag(){
        return tagGoalChooser.getSelected();
    }

    public DoubleTopic getRPiTableTopic(String x){
        return rPiTable.getDoubleTopic(x);
    }

    public boolean pixyCamBroken() {
        return SmartDashboard.getBoolean("Pixy Cam Broken", false);
    }

    public boolean isFieldCentric() {
        return driveModeChooser.getSelected() == DriveMode.FIELD_CENTRIC;      
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

    public GamePiece getGamePiece() {
        return gamePieceChooser.getSelected();
    }

    public ChargeStation getChargeStation(){
        return chargeStationChooser.getSelected();
    }

    public AutoConfiguration getAutoConfiguration() {
        return new AutoConfiguration(
            getStartingGrid(), 
            getStartingNode(),
            getGamePiece(),
            getChargeStation()
        );
    }

    // Create enums for Dashboard elements/parts here
    public static enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }

    public static Dashboard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }
}
