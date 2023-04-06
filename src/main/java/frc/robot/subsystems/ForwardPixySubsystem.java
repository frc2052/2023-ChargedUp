package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pixy.Pixy2;
import frc.robot.pixy.Pixy2CCC;
import frc.robot.pixy.Pixy2CCC.Block;
import frc.robot.pixy.links.SPILink;

public class ForwardPixySubsystem extends SubsystemBase{
    
	private static Pixy2 pixy;

    public ForwardPixySubsystem(){
        pixy = Pixy2.createInstance(new SPILink()); // Creates a new Pixy2 camera using SPILink
		pixy.init(); // Initializes the camera and prepares to send/receive data
    }

    @Override
    public void periodic() {
        findBlocks();
    }

    public void findBlocks(){
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        for (Block block : blocks){
            if (block.getSignature() == 1){
                System.out.println("Cone Position x = " + block.getX() + " y = " + block.getY());
            }
            if (block.getSignature() == 2){
                System.out.println("Cube Position x = " + block.getX() + " y = " + block.getY());
            }
            if (block.getSignature() == 3){
                System.out.println("White Line Position x = " + block.getX() + " y = " + block.getY());
            }
        }
    }

    public Block findCentermostBlock(){
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        Block centerBlock = null;
        for (Block block : blocks){
            if (block.getY() < getMiddleLine().getY()){
                if (centerBlock == null){
                    centerBlock = block;
                } else if (xOffsetFromCenter(block) < xOffsetFromCenter(centerBlock)){
                    centerBlock = block;
                }
            }
        }
        return centerBlock;
    }

    public double xOffsetFromCenter(Block block){
        //pixy cam pixel res width is 316, midpoint is 158
        return (Math.abs(158 - block.getX()));
    }

    public Block getMiddleLine(){
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        Block middleLineBlock = null;
        for (Block block : blocks){
            //pixy cam signature for the white line is 3
            if (block.getSignature() == 3){
                middleLineBlock = block;
            }
        }
        return (middleLineBlock);
    }
}
