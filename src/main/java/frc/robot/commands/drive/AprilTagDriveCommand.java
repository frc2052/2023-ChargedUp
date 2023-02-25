package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class AprilTagDriveCommand extends DriveCommand {
    private final PhotonVisionSubsystem vision;

    // PID constants should be tuned per robot
    private final double LINEAR_P = 0.1;
    private final double LINEAR_I = 0;
    private final double LINEAR_D = 0.0;
    private final PIDController forwardController;
   
    public AprilTagDriveCommand(DrivetrainSubsystem drivetrain, PhotonVisionSubsystem vision) {
        super(drivetrain);
        
        this.vision = vision;

        forwardController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
        forwardController.setTolerance(0.1);

        addRequirements(this.vision);
    }

    @Override
    protected void drive() {
        try {
            drivetrain.drive(
                0,
                forwardController.calculate(
                    PhotonVisionSubsystem.getPose3d(vision.getTarget()).getX(), 
                    0
                ), 
                0, 
                false
            );
        } catch (Exception e) {
            end(true);

            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished(){
        return forwardController.atSetpoint();
    }
}
