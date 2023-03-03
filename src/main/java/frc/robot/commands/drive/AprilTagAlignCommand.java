package frc.robot.commands.drive;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.Auto;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class AprilTagAlignCommand extends DriveCommand {
    private final Node node;

    private final PhotonVisionSubsystem vision;

    // PID constants should be tuned per robot
    private final double TRANSLATION_P = 0.1;
    private final double TRANSLATION_I = 0.0;
    private final double TRANSLATION_D = 0.0;

    private final double ROTATION_P = 0.01;
    private final double ROTATION_I = 0.0;
    private final double ROTATION_D = 0.01;

    private final PIDController translationController;
    private final ProfiledPIDController rotationController;

    private SwerveControllerCommand alignWithTargetCommand;

    public AprilTagAlignCommand(Node node, DrivetrainSubsystem drivetrain, PhotonVisionSubsystem vision) {
        super(drivetrain);
        
        this.node = node;

        this.vision = vision;

        translationController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
        translationController.setTolerance(0.05);

        rotationController = new ProfiledPIDController(
            ROTATION_P, 
            ROTATION_I, 
            ROTATION_D, 
            new TrapezoidProfile.Constraints(0, 0)
        );
        rotationController.setTolerance(0.1);

        addRequirements(this.vision);
    }

    @Override
    protected void drive() {
        try {
            Translation2d robotToTarget = PhotonVisionSubsystem.getRobotToTargetTranslation(vision.getTarget());

            double yOffsetInches = (node.ordinal() - 1) * Auto.NODE_WIDTH_INCHES;

            alignWithTargetCommand = new SwerveControllerCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(robotToTarget.getX(), robotToTarget.getY(), Rotation2d.fromDegrees(-vision.getTarget().getYaw())), 
                    new ArrayList<Translation2d>(), 
                    new Pose2d(0, Units.inchesToMeters(yOffsetInches), new Rotation2d()),
                    new TrajectoryConfig(1, 1)
                ), 
                drivetrain::getPosition,
                DrivetrainSubsystem.getKinematics(),
                translationController,
                translationController,
                rotationController,
                () -> { return new Rotation2d(); },
                drivetrain::setModuleStates, 
                drivetrain
            );
        } catch (Exception e) {
            end(true);
        }
    }

    @Override
    public boolean isFinished(){
        if (alignWithTargetCommand == null) {
            return true;
        }

        return alignWithTargetCommand.isFinished();
    }
}
