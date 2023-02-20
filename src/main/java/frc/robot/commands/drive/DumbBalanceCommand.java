package frc.robot.commands.drive;
import javax.lang.model.util.ElementScanner14;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
public class DumbBalanceCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private double lastPitch = 0;
  private double balanceDegrees = 3;
  private boolean isDriving = false;
  private Timer driveTimer = new Timer();
  private Timer pauseTimer = new Timer();
  private boolean isPaused = false;
  private double maxSpeed = .1;
  private double minSpeed = .075;
  /** Creates a new DumbBalanceCommand. */
  public DumbBalanceCommand(DrivetrainSubsystem drive) {
    this.drivetrain = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = drivetrain.getNavx().getPitch();
    double direction = 1;
    boolean isDropping = (Math.abs(lastPitch) - Math.abs(currentPitch)) > 1.5;
    if (currentPitch > 0)
    {
      direction = -1; //go backwards
    }
    if (Math.abs(currentPitch) < balanceDegrees) //I'm balanced
    {
      System.out.println("I'm Balanced **************************************");
      isDriving = false;
      drivetrain.xWheels();;
      pauseTimer.stop();
      pauseTimer.reset();
  }
    else if (!isDriving)
    {
        System.out.println("Starting to drive **************************************");
      isDriving = true;
      driveTimer.reset();
      driveTimer.start();
      drivetrain.drive(maxSpeed, 0, 0, false);
    }
    else
    {
        System.out.println("I'm already driving **************************************");
      if (isDropping) //starting to tip
      {
        System.out.println("I'm already dropping **************************************");
        pauseTimer.stop();
        pauseTimer.reset();
        drivetrain.xWheels();
      }
      else if (!driveTimer.hasElapsed(.5)) //drive fast for first X seconds
      {
        System.out.println("Driving fast **************************************");
        drivetrain.drive(direction * maxSpeed, 0, 0, false);
      }
      else //drive slow after x seconds until we start to tip
      {
        if (!pauseTimer.hasElapsed(.001)) { //not running
            System.out.println("starting up pause timer **************************************");
            pauseTimer.reset();
            pauseTimer.start();
        }
        if (pauseTimer.hasElapsed(1)){
            if (isPaused){
                System.out.println("driving slow **************************************");
                drivetrain.drive(direction * minSpeed, 0, 0, false);
            }
            else {
                System.out.println("pausing... **************************************");
                drivetrain.xWheels();
            }
            isPaused = !isPaused;
            pauseTimer.stop();
            pauseTimer.reset();
            pauseTimer.start();
        }
        System.out.println("Have a nice day**********************");
      }
    }
    lastPitch = currentPitch;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}