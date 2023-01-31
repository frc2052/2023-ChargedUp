package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PIDCSABCommand extends PIDCommand {
  private  DrivetrainSubsystem drivetrain;
  private double angle = drivetrain.getNavx().getPitch();
  private double speed;

  /** Creates a new PIDCSABCommand. */
  public PIDCSABCommand(
  ) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
