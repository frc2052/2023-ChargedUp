// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class HorizontalAlignmentCommand extends CommandBase {
    private final DoubleSupplier xSupplier;
    private final SlewRateLimiter xLimiter;
    
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final PixySubsystem pixy;

    private final boolean reflective;
    
    private final double TRANSLATION_P = 0.02;
    private final double TRANSLATION_I = 0.0;
    private final double TRANSLATION_D = 0.001;
    
    private final PIDController translationController;

    double coneOffsetDegrees;

    private final Timer ledTimer;

    /** Creates a new HorizontalAlignmentCommand. */
    public HorizontalAlignmentCommand(
        DoubleSupplier xSupplier, 
        DrivetrainSubsystem drivetrain, 
        VisionSubsystem vision,
        PixySubsystem pixy,
        boolean reflective
    ) {
        this.xSupplier = xSupplier;
        
        xLimiter = new SlewRateLimiter(2);

        this.drivetrain = drivetrain;
        this.vision = vision;
        this.pixy = pixy;
        this.reflective = reflective;
        
        translationController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
        translationController.setTolerance(0.75);

        ledTimer = new Timer();

        addRequirements(drivetrain, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pixy.updateConePosition();

        coneOffsetDegrees = Math.toDegrees(Math.atan(pixy.getLastKnownPositionInches() / 32));

        if (reflective) {
            vision.enableLEDs();
            ledTimer.start();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PhotonTrackedTarget target;
        
        if (reflective) {
            target = vision.getReflectiveTarget();
        } else {
            target = vision.getAprilTagTarget();
        }

        double cameraOffsetDegrees = Math.toDegrees(Math.atan(Constants.Camera.CAMERA_POSITION_METERS.getY() / 32));

        System.out.println(cameraOffsetDegrees);

        if (target != null) {
            drivetrain.drive(
                slewAxis(xLimiter, deadBand(-xSupplier.getAsDouble())),
                translationController.calculate(target.getYaw(), cameraOffsetDegrees + coneOffsetDegrees + Dashboard.getInstance().getScoreOffsetDegrees()),
                0,
                false
            );
        } else {
            drivetrain.drive(
                slewAxis(xLimiter, deadBand(-xSupplier.getAsDouble())), 
                translationController.calculate(0, 0),
                0,
                false
            );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDED!");
        drivetrain.stop();
        vision.disableLEDs();
    }

    private double slewAxis(SlewRateLimiter limiter, double value) {
        return limiter.calculate(Math.copySign(Math.pow(value, 2), value));
    }

    private double deadBand(double value) {
        if (Math.abs(value) <= 0.075) {
            return 0.0;
        }
        // Limit the value to always be in the range of [-1.0, 1.0]
        return Math.copySign(Math.min(1.0, Math.abs(value)), value);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        // if (reflective && ledTimer.get() < 0.5) {
        //     return false;
        // } 
        // return translationController.atSetpoint();
    }
}
