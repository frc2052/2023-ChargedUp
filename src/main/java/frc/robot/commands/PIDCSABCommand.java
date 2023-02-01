package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PIDCSABCommand extends PIDCommand {
  private static DrivetrainSubsystem drivetrain;

  private final static double kP = 0.05;
  private final static double kI = 0.05;
  private final static double kD = 0.05;

  public PIDCSABCommand(
    PIDController CSABcontroller,
    DoubleSupplier measurementSource,
    DoubleSupplier setpointSource,
    DoubleConsumer useOutput,
    DrivetrainSubsystem requirements) {
      super(
          // The controller that the command will use
          new PIDController(kP, kI, kD),
          // This should return the measurement
          () -> 0,
          // This should return the setpoint (can also be a constant)
          () -> 0,
          // This uses the output
          output -> drivetrain.drive(new ChassisSpeeds(output, 0, 0))
            // Use the output here
          );
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
      // Configure additional PID options by calling `getController` here.
      getController().setTolerance(1);
      getController().setSetpoint(0);
      getController().calculate(drivetrain.getNavx().getPitch());
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}