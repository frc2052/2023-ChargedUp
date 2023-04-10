// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import frc.robot.auto.common.AutoFactory.ChargeStation;
import frc.robot.auto.common.AutoFactory.GamePiece;
import frc.robot.auto.common.AutoFactory.Grid;
import frc.robot.auto.common.AutoFactory.Node;

/** Add your docs here. */
public class AutoConfiguration {
    private Grid startingGrid;
    private Node startingNode;
    private GamePiece gamePiece;
    private ChargeStation endChargeStation;
    
    public AutoConfiguration(
        Grid startingGrid, 
        Node startingNode,
        GamePiece gamePiece,
        ChargeStation endChargeStation
    ) {
        this.startingGrid = startingGrid;
        this.startingNode = startingNode;
        this.gamePiece = gamePiece;
        this.endChargeStation = endChargeStation;
    }

    public Grid getStartingGrid() {
        return startingGrid;
    }

    public Node getStartingNode() {
        return startingNode;
    }

    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public ChargeStation getChargeStation() {
        return endChargeStation;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof AutoConfiguration) {
            boolean equals = true;
            equals &= ((AutoConfiguration) other).getStartingGrid() == startingGrid;
            equals &= ((AutoConfiguration) other).getStartingNode() == startingNode;
            equals &= ((AutoConfiguration) other).getGamePiece() == gamePiece;
            equals &= ((AutoConfiguration) other).getChargeStation() == endChargeStation;
            return equals;
        }
        return false;
    }
}
