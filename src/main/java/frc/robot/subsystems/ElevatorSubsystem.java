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
    
    private ElevatorPosition previousPosition;
    private ElevatorPosition currentPosition;

    private DigitalInput limitSwitch;
    public ElevatorSubsystem() {
        ErrorCode error;
        
        limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_DIO_CHANNEL);
        
        TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.slot0.kP = Constants.Elevator.BELT_MOTOR_P;
        steerMotorConfiguration.slot0.kI = Constants.Elevator.BELT_MOTOR_I;
        steerMotorConfiguration.slot0.kD = Constants.Elevator.BELT_MOTOR_D;
        
        steerMotorConfiguration.motionCruiseVelocity = Constants.Elevator.BELT_MOTOR_CRUISE_VELOCITY;
        steerMotorConfiguration.motionAcceleration = Constants.Elevator.BELT_MOTOR_MAX_ACCELERATION;

        beltMotor = new TalonFX(Constants.Elevator.BELT_MOTOR);
        beltMotor.configFactoryDefault();

        if ((error = beltMotor.configAllSettings(steerMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError("Failed to configure belt motor: " + error.toString(), false);
        }
        
        beltMotor.setNeutralMode(NeutralMode.Brake);
        beltMotor.setInverted(true);

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

        if (!limitSwitch.get()) {
            zeroEncoder();

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
        FLOORCUBE(10000),
        FLOORCONE(20155),
        BABYBIRD(23963),
        MIDSCORE(86590),
        TOPSCORE(117875);

        private final int positionTicks;

        private ElevatorPosition(int positionTicks) {
            this.positionTicks = positionTicks;
        }

        public int getPositionTicks() {
            return positionTicks;
        }
    }
}
