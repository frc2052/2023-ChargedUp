package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class AprilTagAlignCommand extends DriveCommand {
    private final PhotonVisionSubsystem vision;

    // PID constants should be tuned per robot
    private final double LINEAR_P = 0.1;
    private final double LINEAR_I = 0;
    private final double LINEAR_D = 0.0;

    private final PIDController xController;
    private final PIDController yController;
   
    public AprilTagAlignCommand(DrivetrainSubsystem drivetrain, PhotonVisionSubsystem vision) {
        super(drivetrain);
        
        this.vision = vision;

        xController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
        xController.setTolerance(0.1);

        yController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
        yController.setTolerance(0.1);

        addRequirements(this.vision);
    }

    @Override
    protected void drive() {
        try {
            Translation2d robotToTarget = PhotonVisionSubsystem.getRobotToTargetTranslation(vision.getTarget());

            drivetrain.drive(
                xController.calculate(robotToTarget.getX(), 0),
                yController.calculate(robotToTarget.getY(), 0), 
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
        return xController.atSetpoint() && yController.atSetpoint();
    }
}
