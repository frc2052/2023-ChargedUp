//Simple Charge Station Auto-Balance Command
    
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class ChargeStationAutoBalCommand extends CommandBase {
private DrivetrainSubsystem drivetrain;
private double angle;
private double maxMetersPerSecond;
private double maxReverseMetersPerSecond;
private double deadzone;
private double speed;

    public ChargeStationAutoBalCommand(
        DrivetrainSubsystem drivetrain,
        double maxMetersPerSecond,
        double maxReverseMetersPerSecond,
        double deadzone)
        {
        this.drivetrain = drivetrain;
        this.maxMetersPerSecond = maxMetersPerSecond;
        this.maxReverseMetersPerSecond = maxReverseMetersPerSecond;
        this.deadzone = deadzone;

        addRequirements(this.drivetrain);
    }

    //Initialize
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //If the given angle is larger than deadzone, calculate speed
        if (drivetrain.getNavx().getPitch() > deadzone){
            angle = drivetrain.getNavx().getPitch();
            //Calculates speed needed from angle
            //low2 + (value - low1) * (high2 - low2) / (high1 - low1)
            speed = maxReverseMetersPerSecond + (angle - -15) * (maxMetersPerSecond - maxReverseMetersPerSecond) / (15 - -15); 
        }
        else{
            speed = 0;
        }
  
        drivetrain.drive(new ChassisSpeeds(speed, 0, 0));
        
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
