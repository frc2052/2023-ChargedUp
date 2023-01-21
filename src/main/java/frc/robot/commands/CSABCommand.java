//Charge Station Auto-Balance Command


    
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivetrain;


public class CSABCommand extends CommandBase {
private Drivetrain drivetrain;
private double angle;
private double maxSpeed;
private double minSpeed;
private double delay;
private double deadzone;
private double speed;

    public CSABCommand(
        Drivetrain drivetrain,
        double angle,
        double maxSpeed,
        double minSpeed,
        double delay,
        double deadzone,
        double speed)
        {
        this.drivetrain = drivetrain;
        this.angle = angle;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.delay = delay;
        this.deadzone = deadzone;
        this.speed = speed;
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
        speed = -15 + (angle - minSpeed) * (15 - -15) / (maxSpeed - minSpeed);
        System.out.println(speed);
    
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
