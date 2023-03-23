// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SmartHorizontalAlignmentCommand extends DriveCommand {
    private final VisionSubsystem vision;
    private final PixySubsystem pixy;
    
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier rotationSupplier;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter rotationLimiter;

    private final PIDController yController;

    private final Timer ledEnableTimer;

    /** Creates a new SmartHorizontalAlignmentCommand. */
    public SmartHorizontalAlignmentCommand(
        DrivetrainSubsystem drivetrain, 
        VisionSubsystem vision,
        PixySubsystem pixy,
        DoubleSupplier xSupplier,
        DoubleSupplier rotationSupplier
    ) {
        super(drivetrain);

        this.vision = vision;
        this.pixy = pixy;

        this.xSupplier = xSupplier;
        this.rotationSupplier = rotationSupplier;

        xLimiter = new SlewRateLimiter(2);
        rotationLimiter = new SlewRateLimiter(2);

        yController = new PIDController(0.02, 0, 0.001);
        yController.setTolerance(0.5);

        ledEnableTimer = new Timer();

        addRequirements(this.vision, this.pixy);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pixy.updateConePosition();

        vision.enableLEDs();
        ledEnableTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void drive() {
        PhotonTrackedTarget target = vision.getReflectiveTarget();

        if (target != null) {
            // Total offset of the cone in the intake, the camera's mounted position on the robot, and the manual dashboard offset.
            double coneOffsetInches = pixy.getLastKnownPositionInches() + Units.metersToInches(Constants.Camera.CAMERA_POSITION_METERS.getY()) + Dashboard.getInstance().getScoreOffsetDegrees();

            // Finds the PID target angle in degrees given a constant distance of 36 inches and the offset of the cone in the intake.
            double cameraConeOffsetDegrees = Math.copySign(-Math.toDegrees(Math.atan(Units.inchesToMeters(36) / Math.abs(Units.inchesToMeters(coneOffsetInches)))) + 90, coneOffsetInches);

            System.out.println(cameraConeOffsetDegrees);

            drivetrain.drive(
                slewAxis(xLimiter, deadBand(-xSupplier.getAsDouble())),
                yController.calculate(target.getYaw(), cameraConeOffsetDegrees),
                slewAxis(rotationLimiter, deadBand(-rotationSupplier.getAsDouble() * .75)),
                false
            );
        } else {
            drivetrain.drive(
                slewAxis(xLimiter, deadBand(-xSupplier.getAsDouble())),
                yController.calculate(0, 0),
                slewAxis(rotationLimiter, deadBand(-rotationSupplier.getAsDouble() * .75)),
                false
            );
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Gives enough time to enable LEDs and track a target
        if (ledEnableTimer.get() <= 1) {
            return false;
        }

        return yController.atSetpoint();
    }
}
