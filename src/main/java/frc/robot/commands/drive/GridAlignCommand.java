package frc.robot.commands.drive;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.Auto;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GridAlignCommand extends CommandBase {
    private final Node node;

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;

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

    public GridAlignCommand(
        Node node, 
        DrivetrainSubsystem drivetrain, 
        VisionSubsystem vision,
        PixySubsystem pixy
    ) {
        this.node = node;

        this.drivetrain = drivetrain;
        this.vision = vision;

        translationController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);

        rotationController = new ProfiledPIDController(
            ROTATION_P, 
            ROTATION_I, 
            ROTATION_D, 
            new TrapezoidProfile.Constraints(0, 0)
        );
        rotationController.setTolerance(0.1);

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        PhotonTrackedTarget target = vision.getAprilTagTarget();

        Translation2d robotToTarget = VisionSubsystem.getRobotToTargetTranslationMeters(target);
            
        double yOffsetInches = (node.ordinal() - 1) * Auto.NODE_WIDTH_INCHES;

        if (target != null) {
            alignWithTargetCommand = new SwerveControllerCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(robotToTarget.getX(), robotToTarget.getY(), new Rotation2d()), 
                    new ArrayList<Translation2d>(), 
                    new Pose2d(0, Units.inchesToMeters(yOffsetInches), new Rotation2d()),
                    new TrajectoryConfig(1, 1)
                ), 
                drivetrain::getPosition,
                DrivetrainSubsystem.getKinematics(),
                translationController,
                translationController,
                rotationController,
                () -> { 
                    return new Rotation2d();
                    //return Rotation2d.fromDegrees(-target.getYaw());   
                },
                drivetrain::setModuleStates, 
                drivetrain
            );
        }

        // try {
        //     Translation2d robotToTarget = VisionSubsystem.getRobotToTargetTranslation(vision.getTarget());

        //     double yOffsetInches = (node.ordinal() - 1) * Auto.NODE_WIDTH_INCHES;

        //     alignWithTargetCommand = new SwerveControllerCommand(
        //         TrajectoryGenerator.generateTrajectory(
        //             new Pose2d(robotToTarget.getX(), robotToTarget.getY(), new Rotation2d()), 
        //             new ArrayList<Translation2d>(), 
        //             new Pose2d(0, Units.inchesToMeters(yOffsetInches), new Rotation2d()),
        //             new TrajectoryConfig(1, 1)
        //         ), 
        //         drivetrain::getPosition,
        //         DrivetrainSubsystem.getKinematics(),
        //         translationController,
        //         translationController,
        //         rotationController,
        //         () -> { 
        //             try {
        //                 return Rotation2d.fromDegrees(-vision.getTarget().getYaw());
        //             } catch(TargetNotFoundException e) {
        //                 return new Rotation2d();
        //             }
        //          },
        //         drivetrain::setModuleStates, 
        //         drivetrain
        //     );
        // } catch (Exception e) {
        //     end(true);
        // }
    }

    @Override
    public boolean isFinished(){
        if (alignWithTargetCommand == null) {
            return true;
        }

        return alignWithTargetCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
