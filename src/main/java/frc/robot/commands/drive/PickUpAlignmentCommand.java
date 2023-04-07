// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePixySubsystem;

public class PickUpAlignmentCommand extends DriveCommand {
    private final IntakePixySubsystem pixy;
    
    private final PIDController yController;

    /** Creates a new DumbHorizontalAlignmentCommand. */
    public PickUpAlignmentCommand(
        DoubleSupplier xSupplier,
        DoubleSupplier rotationSupplier,
        DrivetrainSubsystem drivetrain,
        IntakePixySubsystem pixy
    ) {
        super(xSupplier, () -> 0, rotationSupplier, Dashboard.getInstance()::isFieldCentric, drivetrain);

        this.pixy = pixy;

        yController = new PIDController(1, 0, 0);
        yController.setSetpoint(0);
        yController.setTolerance(0.5);

        addRequirements(this.pixy);
    }

    @Override
    protected double getY() {
       

        //return -yController.calculate() / 75.76;
        return 0;
    }

    @Override
    protected double getRotation() {
        return super.getRotation() * 0.25;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
