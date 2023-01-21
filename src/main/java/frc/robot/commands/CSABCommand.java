//Charge Station Auto-Balance Command


    
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class CSABCommand extends CommandBase {
private DrivetrainSubsystem drivetrain;
private double angle;
private double maxSpeed;
private double minSpeed;
private double delay;
private double deadzone;
private double speed;

    public CSABCommand(
        DrivetrainSubsystem drivetrain,
        double angle,
        double maxSpeed,
        double minSpeed,
        double delay,
        double deadzone,
        double speed)
        {
        this.angle = angle;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.delay = delay;
        this.deadzone = deadzone;
        this.speed = speed;

        addRequirements(drivevtrain);
    }

    //Initialize
    @Override
    public void initialize() {
        minSpeed = -1;
        maxSpeed = 1;
        angle = -3;
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //low2 + (value - low1) * (high2 - low2) / (high1 - low1)
        speed = minSpeed + (angle - -15) * (maxSpeed - minSpeed) / (15 - -15);
        //speed = -15 + (angle - minSpeed) * (15 - -15) / (maxSpeed - minSpeed);
    
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
