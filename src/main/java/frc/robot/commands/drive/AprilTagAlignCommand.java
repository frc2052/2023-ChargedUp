package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Auto;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class AprilTagAlignCommand extends DriveCommand {
    private final Node node;

    private final PhotonVisionSubsystem vision;

    // PID constants should be tuned per robot
    private final double LINEAR_P = 0.1;
    private final double LINEAR_I = 0.0;
    private final double LINEAR_D = 0.0;

    private final double ROTATION_P = 0.01;
    private final double ROTATION_I = 0.0;
    private final double ROTATION_D = 0.01;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public AprilTagAlignCommand(Node node, DrivetrainSubsystem drivetrain, PhotonVisionSubsystem vision) {
        super(drivetrain);
        
        this.node = node;

        this.vision = vision;

        xController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
        xController.setTolerance(0.05);

        yController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
        yController.setTolerance(0.05);

        rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        rotationController.setTolerance(0.05);

        addRequirements(this.vision);
    }

    @Override
    protected void drive() {
        try {
            Translation2d robotToTarget = PhotonVisionSubsystem.getRobotToTargetTranslation(vision.getTarget());

            double yOffset = (node.ordinal() - 1) * Auto.NODE_WIDTH_INCHES;

            drivetrain.drive(
                0, // -xController.calculate(robotToTarget.getX(), Units.inchesToMeters(Constants.Auto.ROBOT_LENGTH_INCHES)),
                0, // -yController.calculate(robotToTarget.getY(), yOffset), 
                -rotationController.calculate(-vision.getTarget().getYaw(), 0), 
                false
            );
        } catch (Exception e) {
            end(true);
        }
    }

    @Override
    public boolean isFinished(){
        return xController.atSetpoint() && yController.atSetpoint();
    }
}
