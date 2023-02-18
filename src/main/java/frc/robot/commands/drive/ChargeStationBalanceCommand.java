package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChargeStationBalanceCommand extends DriveCommand {    
    private final PIDController balanceController;

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
    protected void drive() {
        double output = balanceController.calculate(drivetrain.getNavx().getPitch(), 0);

        Dashboard.getInstance().putData("Balance Drive", Math.copySign(
            Math.min(Math.abs(output), Constants.AutoBalance.MAX_SPEED_METERS_PER_SECOND), 
            output
        ));

        drivetrain.drive(
            Math.copySign(
                Math.min(Math.abs(output), Constants.AutoBalance.MAX_SPEED_METERS_PER_SECOND), 
                output
            ),
            0,
            0, 
            false
        );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}