package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.PhotonVision;

public class AprilTagD extends CommandBase{
    private PhotonVision m_vision;
    private DrivetrainSubsystem m_driveTrain;
    private double visionRotation = 0;
    private double horizontalAngle;
    private boolean isLinedUp;

    public AprilTagD(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               PhotonVision vision,
                               DashboardSubsystem dashboard) {
        super(drivetrainSubsystem,
        translationXSupplier,
        translationYSupplier,
        () -> { return 0.0; }, //this value will not be used because getTurnWillBeOverriden
        dashboard);

        this.m_vision = vision;
        this.m_driveTrain = drivetrainSubsystem;
    }
    public boolean getIsLinedUp() {
        return isLinedUp;
    }


       // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(0);

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_I = 0;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_I = 0;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);


    // Drive motors
    PWMVictorSPX leftMotor = new PWMVictorSPX(0);
    PWMVictorSPX rightMotor = new PWMVictorSPX(1);
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    @Override
    public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed;

        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

                // Also calculate angular power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                forwardSpeed = 0;
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            forwardSpeed = -xboxController.getRightY();
            rotationSpeed = xboxController.getLeftX();
        }

        // Use our forward/turn speeds to control the drivetrain
        drive.arcadeDrive(forwardSpeed, rotationSpeed);
    }
    
}
