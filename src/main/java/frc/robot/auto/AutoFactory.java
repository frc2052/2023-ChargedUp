// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

/** Add your docs here. */
public class AutoFactory {
    
    public AutoFactory(){
        
    }
    public SequentialCommandGroup getAuto(Grid grid, Node node){
        double xMeters = Units.inchesToMeters((3.5 + 16.5)+(3*grid.ordinal()+node.ordinal())*(18.5 + 13.5));
        double yMeters = (Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS+(2*Constants.Auto.BUPPER_DEPTH_METERS))/2;
        Pose2d startingPose = new Pose2d(xMeters, yMeters,new Rotation2d());
         

        
        return null;


    }
    public static enum Grid{
        LEFT_GRID,
        CO_OP,
        RIGHT_GRID, 
    }
    public static enum Node{
        LEFT_CONE, 
        MIDDLE_CONE, 
        RIGHT_CUBE;
        
    }
}
