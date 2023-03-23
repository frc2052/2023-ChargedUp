// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DumbHorizontalAlignmentCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final PixySubsystem pixy;
    
    private final PIDController yController;

    private final Timer ledEnableTimer;

    /** Creates a new DumbHorizontalAlignmentCommand. */
    public DumbHorizontalAlignmentCommand(
        DrivetrainSubsystem drivetrain, 
        VisionSubsystem vision,
        PixySubsystem pixy
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.pixy = pixy;

        yController = new PIDController(0.02, 0, 0.001);
        yController.setTolerance(0.75);

        ledEnableTimer = new Timer();

        addRequirements(this.drivetrain, this.vision, this.pixy);
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
    public void execute() {
        // Max target yaw
        double minConeAlignedYawDegrees = -3;
        double maxConeAlignedYawDegrees = 12;

        double conePositionInches = pixy.getLastKnownPositionInches() + 6;

        PhotonTrackedTarget target = vision.getReflectiveTarget();

        if (target != null) {
            double goalYaw = (conePositionInches * (maxConeAlignedYawDegrees - minConeAlignedYawDegrees)) / 12;

            drivetrain.drive(
                0,
                yController.calculate(target.getYaw(), goalYaw),
                0,
                false
            );
        } else {
            System.out.println("No target!");
            drivetrain.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (ledEnableTimer.get() <= 0.5) {
            return false;
        }

        return false;
    }
}
