package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;


public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX beltMotor;
    
    private final DigitalInput limitSwitch;

    private ElevatorPosition previousPosition;
    private ElevatorPosition currentPosition;

    public ElevatorSubsystem() {
        ErrorCode error;
        
        TalonFXConfiguration beltMotorConfiguration = new TalonFXConfiguration();
        beltMotorConfiguration.slot0.kP = Constants.Elevator.BELT_MOTOR_P;
        beltMotorConfiguration.slot0.kI = Constants.Elevator.BELT_MOTOR_I;
        beltMotorConfiguration.slot0.kD = Constants.Elevator.BELT_MOTOR_D;
        
        // Set motion magic cruise velocity and max acceleration.
        beltMotorConfiguration.motionCruiseVelocity = Constants.Elevator.BELT_MOTOR_CRUISE_VELOCITY;
        beltMotorConfiguration.motionAcceleration = Constants.Elevator.BELT_MOTOR_MAX_ACCELERATION;

        beltMotor = new TalonFX(Constants.Elevator.BELT_MOTOR);
        beltMotor.configFactoryDefault();

        if ((error = beltMotor.configAllSettings(beltMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError("Failed to configure belt motor: " + error.toString(), false);
        }
        
        beltMotor.setNeutralMode(NeutralMode.Brake);
        beltMotor.setInverted(true);

        limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_DIO_CHANNEL);

        previousPosition = ElevatorPosition.STARTING;
        currentPosition = ElevatorPosition.STARTING;

        // Assume the elevator will start at the lowest possible position.
        zeroEncoder();
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(
            Constants.Dashboard.ELEVATOR_POSITION_KEY, 
            beltMotor.getSelectedSensorPosition()
        );

        // Limit switch returns true by default.
        if (!limitSwitch.get() || beltMotor.getSelectedSensorPosition() < 0) {
            zeroEncoder();

            // If the elevator is traveling downwards stop the belt motor and end the current command.
            if (currentPosition.getPositionTicks() < previousPosition.getPositionTicks()) {
                stop();
                getCurrentCommand().cancel();
            }
        }
    }

    public void setPosition(ElevatorPosition elevatorPosition) {
        previousPosition = currentPosition;
        currentPosition = elevatorPosition;
        
        // If the arm mechanism drags down the belt due to gravity add an arbitrary feed forward constant.
        beltMotor.set(
            ControlMode.MotionMagic,
            elevatorPosition.getPositionTicks()
        );
    }

    public boolean atPosition() {
        return Math.abs(
            currentPosition.getPositionTicks() - beltMotor.getSelectedSensorPosition()
        ) <= Constants.Elevator.BELT_MOTOR_DEAD_ZONE_TICKS;
    }

    public void zeroEncoder() {
        ErrorCode error;
        if ((error = beltMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) {
            DriverStation.reportError(
                "Failed to set belt motor encoder position: " + error.toString(), 
                false
            );
        }
    }

    public void manualUp() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.1);
    }

    public void manualDown() {
        if (limitSwitch.get()) {
            beltMotor.set(TalonFXControlMode.PercentOutput, -0.075);
        }
    }

    public void stop() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public static enum ElevatorPosition {
        STARTING(0),
        FLOOR_CUBE(10000),
        FLOOR_CONE(20155),
        BABY_BIRD(9500),
        MID_SCORE(91500),
        TOP_SCORE(122000);

        private final int positionTicks;

        private ElevatorPosition(int positionTicks) {
            this.positionTicks = positionTicks;
        }

        public int getPositionTicks() {
            return positionTicks;
        }
    }
}
