package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GamePieceAlignmentCommand extends DriveCommand {
    private final ForwardPixySubsystem pixy;
    private final IntakeSubsystem intake;

    // Mount offset is 3 Inches.
    private final double xMountOffsetPixels = 0;

    private final PIDController xController;
    private final PIDController yController;

    public GamePieceAlignmentCommand(
        DoubleSupplier goalX,
        ForwardPixySubsystem pixy,
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake
    ) {
        super(() -> 0, () -> 0, () -> 0, () -> false, drivetrain);

        this.pixy = pixy;
        this.intake = intake;

        xController = new PIDController(0.3, 0, 0);
        xController.setTolerance(0.1);
        xController.setSetpoint(goalX.getAsDouble());

        yController = new PIDController(1.25, 0, 0);
        yController.setTolerance(10);
        yController.setSetpoint(-xMountOffsetPixels);

        addRequirements(pixy, drivetrain);
    }

    @Override
    protected double getY() {
        return yController.calculate(pixy.xOffsetFromCenter(pixy.findCentermostBlock())) / 158;
    }

    @Override
    protected double getX() {
        return xController.calculate(drivetrain.getPosition().getX());
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() || intake.isCurrentLimiting();
    }
}
