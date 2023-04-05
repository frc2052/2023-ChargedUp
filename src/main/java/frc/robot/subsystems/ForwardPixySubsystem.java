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
        getBiggestBlock();
    }

    public static Block getBiggestBlock() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 25);
		System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
		}
        System.out.println("Found a block x = " + largestBlock.getX() + " y  = " + largestBlock.getY());
		return largestBlock;
	}
    
}
