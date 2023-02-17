// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem.TargetNotFoundException;



public class GridAlignmentCommand extends DriveCommand {
    private final PhotonVisionSubsystem vision;

    // TODO: Use preset autonomous translation PIDController and rotation PIDController from autos
    private final ProfiledPIDController horizontalController;
    
    /** Creates a new GridAlignmentCommand. */
    public GridAlignmentCommand(DrivetrainSubsystem drivetrain, PhotonVisionSubsystem vision) {
        super(drivetrain);

        this.vision = vision;

        horizontalController = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        horizontalController.setTolerance(0.1);

        addRequirements(this.vision);
    }

    @Override
    protected void drive() {
        try {
            drivetrain.drive(
                horizontalController.calculate(PhotonVisionSubsystem.getHorizontalOffsetMeters(vision.getTarget()), 0),
                0,
                0,
                false
            );
        } catch (TargetNotFoundException e) {
            end(true);

            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return horizontalController.atSetpoint();
    }
}
