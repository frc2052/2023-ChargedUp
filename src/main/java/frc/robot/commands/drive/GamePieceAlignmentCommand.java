package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;

public class GamePieceAlignmentCommand extends DriveCommand {
    private final ForwardPixySubsystem pixy;
    private final PIDController yController;

    public GamePieceAlignmentCommand(
        ForwardPixySubsystem pixy,
        DrivetrainSubsystem drivetrain
    ) {
        super(() -> 0, () -> 0, () -> 0, Dashboard.getInstance()::isFieldCentric, drivetrain);

        this.pixy = pixy;

        yController = new PIDController(0.5, 0, 0);
        yController.setTolerance(10);
        yController.setSetpoint(0);

        addRequirements(pixy, drivetrain);
    }

    @Override
    protected double getY() {
        double x = -yController.calculate(pixy.xOffsetFromCenter(pixy.findCentermostBlock())) / 158;
        //System.out.println(x);
        return x;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
