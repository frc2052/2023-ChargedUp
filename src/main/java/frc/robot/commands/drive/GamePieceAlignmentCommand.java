package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;

public class GamePieceAlignmentCommand extends CommandBase{
    private final ForwardPixySubsystem pixy;
    private final DrivetrainSubsystem drivetrain;
    private final PIDController yController;

    public GamePieceAlignmentCommand(
        ForwardPixySubsystem pixy,
        PIDController yController,
        DrivetrainSubsystem drivetrain
    ) {
        this.pixy = pixy;
        this.yController = yController;
        this.drivetrain = drivetrain;

        yController = new PIDController(0.01, 0, 0);
        yController.setTolerance(10);

        addRequirements(pixy, drivetrain);
    }

    @Override
    public void execute(){
        drivetrain.drive(
            0, 
            yController.calculate(pixy.findCentermostBlock().getX(), 0) / 316,
            0,
            false
            );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
