package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;


public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX beltMotor;

    private ElevatorPosition currentDesiredPosition;

    public ElevatorSubsystem() {
        ErrorCode error;
        
        TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.slot0.kP = Constants.Elevator.BELT_MOTOR_P;
        steerMotorConfiguration.slot0.kI = Constants.Elevator.BELT_MOTOR_I;
        steerMotorConfiguration.slot0.kD = Constants.Elevator.BELT_MOTOR_D;
        
        // TODO: test and change values accordingly.
        steerMotorConfiguration.motionCruiseVelocity = Constants.Elevator.TICKS_PER_ROTATION;
        steerMotorConfiguration.motionAcceleration = Constants.Elevator.TICKS_PER_ROTATION / 2;

        beltMotor = new TalonFX(Constants.Elevator.BELT_MOTOR);
        if ((error = beltMotor.configAllSettings(steerMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError("Failed to configure belt motor: " + error.toString(), false);
        }
        beltMotor.setNeutralMode(NeutralMode.Brake);

        // Assume the elevator will start at the lowest possible position.
        if ((error = beltMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to set belt motor encoder position: " + error.toString(), 
                false
            );
        }
    }

    @Override
    public void periodic() {
        // If the elevator ever tried to extend past a maximum limit immediately cancel the command
        // trying to push it past that limit and stop the elevator.
        if (Math.abs(beltMotor.getSelectedSensorPosition()) > Constants.Elevator.MAX_POSITION_TICKS) {
            stop();
            this.getCurrentCommand().cancel();
        }

        Dashboard.getInstance().putData(
            Constants.Dashboard.ELEVATOR_POSITION_KEY, 
            beltMotor.getSelectedSensorPosition()
        );
    }

    public void setPosition(ElevatorPosition elevatorPosition) {
        currentDesiredPosition = elevatorPosition;
        
        // If the arm mechanism drags down the belt due to gravity add an arbitrary feed forward constant.
        beltMotor.set(
            ControlMode.MotionMagic,
            elevatorPosition.getPositionTicks()
        );
    }

    public boolean atPosition() {
        return beltMotor.getSelectedSensorPosition() == currentDesiredPosition.getPositionTicks();
    }

    public void manualUp() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.05);
    }

    public void manualDown() {
        beltMotor.set(TalonFXControlMode.PercentOutput, -0.05);
    }

    public void stop() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public static enum ElevatorPosition {
        GROUND_PICK_UP(0),
        BOTTOM_ROW(0),
        MIDDLE_ROW(500),
        TOP_ROW(1000);

        private final int positionTicks;

        private ElevatorPosition(int positionTicks) {
            this.positionTicks = positionTicks;
        }

        public int getPositionTicks() {
            return positionTicks;
        }
    }
}
