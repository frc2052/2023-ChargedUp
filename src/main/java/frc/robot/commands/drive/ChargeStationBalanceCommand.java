package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChargeStationBalanceCommand extends DriveCommand {    
    private final PIDController balanceController;

    private boolean holding = false;
    private Timer balanceTimer;

    public ChargeStationBalanceCommand(DrivetrainSubsystem drivetrain) {
        super(drivetrain);

        balanceController = new PIDController(
            Constants.AutoBalance.BALANCE_P,
            Constants.AutoBalance.BALANCE_I,
            Constants.AutoBalance.BALANCE_D
        );
        balanceController.setSetpoint(0);
        balanceController.setTolerance(Constants.AutoBalance.BALANCE_TOLERANCE_DEGREES);
    }

    @Override
    public void initialize(){
        holding = false;
    }

    @Override
    protected void drive() {
        double output = balanceController.calculate(drivetrain.getNavx().getPitch(), 0);

        Dashboard.getInstance().putData(
            "Balance Drive",
            drivetrain.getNavx().getPitch()
            // Math.copySign(
            //     Math.min(Math.abs(output), Constants.AutoBalance.MAX_SPEED_METERS_PER_SECOND), 
            //     output
            // )
        );

        if (Math.abs(drivetrain.getNavx().getPitch()) > 5 && !holding) {
            drivetrain.drive(
                Math.copySign(0.1, (double) -(drivetrain.getNavx().getPitch())),
                0,
                0, 
                false
            );
        } else {
            if (balanceTimer.hasElapsed(2)){
                drivetrain.xWheels();
                holding = true;
            } else if ((balanceTimer.hasElapsed(0.1) && drivetrain.getNavx().getPitch() < 14)){
                balanceTimer.start();
            } else if (drivetrain.getNavx().getPitch() > 14){
                balanceTimer.stop();
                balanceTimer.reset();
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}