// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DumbHorizontalAlignmentCommand extends DriveCommand {
    private final VisionSubsystem vision;
    private final PixySubsystem pixy;
    
    private final DoubleSupplier xSupplier;
    private final SlewRateLimiter xLimiter;
    
    private final PIDController yController;

    private final Timer ledEnableTimer;

    /** Creates a new DumbHorizontalAlignmentCommand. */
    public DumbHorizontalAlignmentCommand(
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
        xLimiter = new SlewRateLimiter(2);

        yController = new PIDController(1, 0, 0);
        yController.setTolerance(0.5);

        ledEnableTimer = new Timer();

        addRequirements(this.vision, this.pixy);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        vision.enableLEDs();
        ledEnableTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void drive() {
        // Max target yaw
        double minConeAlignedYawDegrees = -3.8;
        double maxConeAlignedYawDegrees = 11.7;

        double conePositionInches = pixy.getLastKnownPositionInches() + 6;
        double conePosPct = conePositionInches / 12;

        PhotonTrackedTarget target = vision.getReflectiveTarget();

        double angleRange = maxConeAlignedYawDegrees - minConeAlignedYawDegrees;
        double offsetAngle = angleRange * conePosPct;
        double goalYaw = minConeAlignedYawDegrees + offsetAngle;

        if (target != null) {
            double targetYaw = target.getYaw();

            drivetrain.drive(
                slewAxis(xLimiter, deadBand(xSupplier.getAsDouble())),
                // Normalize to our camera FOV.
                yController.calculate(targetYaw, goalYaw) / 75.76,
                0,
                false
            ); 
        } else {
            System.out.println("No target!");

            drivetrain.drive(
                slewAxis(xLimiter, deadBand(xSupplier.getAsDouble())),
                yController.calculate(0, 0),
                0,
                false
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        vision.disableLEDs();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
