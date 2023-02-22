// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.field;

import frc.robot.io.Dashboard.Channel;
import frc.robot.io.Dashboard.GamePiece;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;

/** Add your docs here. */
public class DynamicAutoConfiguration {
    private final Grid startingGrid;
    private final Node startingNode; 
    private final Channel exitChannel;
    private final GamePiece gamePiece;
    private final boolean scoreGamePiece;
    private final Grid scoreGrid;
    private final Node scoreNode;
    private final Channel enterChannel;
    private final boolean endChargeStation;
    
    public DynamicAutoConfiguration(
        Grid startingGrid, 
        Node startingNode, 
        Channel exitChannel, 
        GamePiece gamePiece,
        boolean scoreGamePiece,
        Grid scoreGrid,
        Node scoreNode,
        Channel enterChannel,
        boolean endChargeStation
    ) {
        this.startingGrid = startingGrid;
        this.startingNode = startingNode;
        this.exitChannel = exitChannel;
        this.gamePiece = gamePiece;
        this.scoreGamePiece = scoreGamePiece;
        this.scoreGrid = scoreGrid;
        this.scoreNode = scoreNode;
        this.enterChannel = enterChannel;
        this.endChargeStation = endChargeStation;
    }

    public Grid getStartingGrid() {
        return startingGrid;
    }

    public Node getStartingNode() {
        return startingNode;
    }

    public Channel getExitChannel() {
        return exitChannel;
    }

    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public boolean scoreGamePiece() {
        return scoreGamePiece;
    }

    public Grid getScoreGrid() {
        return scoreGrid;
    }

    public Node getScoreNode() {
        return scoreNode;
    }

    public Channel getEnterChannel() {
        return enterChannel;
    }

    public boolean endChargeStation() {
        return endChargeStation;
    }
}