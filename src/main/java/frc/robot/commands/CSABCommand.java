//Charge Station Auto-Balance Command

    /* Thinking chair:
    Tip protection: (if beyond a degree then drive in opposite direction)
    If angle > about +15 degrees then move at 'x' m/s. 
    If angle > about -15 degrees then move at '-x' m/s. 
    adjust tiny bit after huge charge station tipping 
    when tips forward, drive backwards tiny bit
    get where want x swerve drive (hold off on this idea)
    main goal: how fast go?, adjustable deadzone, max/mim speed, mim < s < max 
    have short delay (1 sec may to much, maybe not enough)
    dont go over edge (side to side)
    x out swerve
    proof of concept
    remapping (t = (value-low1)/(high1-low1))
    get angle from gyro

    Sudo example: dosnt have a standard, like a function not a whole page of code, ignore main repetive stuff, make up as go,
    simpilar make problem easier it is to do, dont overhtink it 
    ----import----
    get drivetrain SS
    Get Gryo
    ----class----
    CSABCommand
ignore public, private, ect
substem has soem commands

    better to have to much and delete later
    double angle
    double maxSpeed
    double minSpeed
    double delay
    double deadzone

    constrructor(--Variables--)
    
    intiaize()
 most code go here: exucute()
    End()
    never stop balancing unless driver said so

    -1(min) to 1(max) speed
    autobalance(angle,max,min)
       *** get angle, speed = remap anlge from minS to maxS
        return speed;

    get drivetrain subsystem
    get gyro subsystem


    calculateSpeed
        get gyro angle (math) = speed
    
    
    */
    
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
drivetrain 
GyroSubsystem


public class CSABCommand extends CommandBase {

    public CSABCommand(){
        
    }

    //Initialize
    @Override
    public void initialize() {
        
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
    
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
