package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChargeStationBalanceCommand extends PIDCommand {
    private final static double kP = 0.01;
    private final static double kI = 0.0;
    private final static double kD = 0.0;

    private static final double SPEED_LIMIT_MPS = 0.05;
    

    private DrivetrainSubsystem drivetrain; 

    public ChargeStationBalanceCommand(
        DrivetrainSubsystem drivetrain
    ) {
        super(
            // The controller that the command will use
            new PIDController(kP, kI, kD),
            // This should return the measurement
            () -> drivetrain.getNavx().getPitch(),
            // This should return the setpoint (can also be a constant)
            () -> 0,
            // This uses the output
            output -> {
                drivetrain.drive((Math.max(Math.min(output, SPEED_LIMIT_MPS),-SPEED_LIMIT_MPS)), 0, 0, false);
                System.out.println("Output: " + output);
                System.out.println("Pitch:  " + drivetrain.getNavx().getPitch());
            }
            // Use the output here
        );

        getController().setSetpoint(0);

        this.drivetrain = drivetrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        // Configure additional PID options by calling `getController` here.
        getController().setTolerance(14);
    }

    // Returns true when the command should end.
    
    @Override
    public boolean isFinished() {
        return false;
    }
}