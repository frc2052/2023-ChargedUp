package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.music.Orchestra;

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

    private final Orchestra warningSound;

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

        warningSound = new Orchestra(List.of(beltMotor));

        // Assume the elevator will start at the lowest possible position.
        zeroEncoder();
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(
            Constants.Dashboard.ELEVATOR_POSITION_KEY, 
            beltMotor.getSelectedSensorPosition()
        );

        Dashboard.getInstance().putData(
            Constants.Dashboard.ELEVATOR_LIMIT_SWITCH_KEY, 
            elevatorZeroed()
        );

        // For testing: play warning sound if elevator isn't zeroed but the encoder is.
        if (!elevatorZeroed() && beltMotor.getSelectedSensorPosition() <= 0) {
            if (!warningSound.isPlaying()) {
                // warningSound.loadMusic("warning.mid");
                // warningSound.play();
            }
        } else {
            // warningSound.stop();
        }

        if (elevatorZeroed() || beltMotor.getSelectedSensorPosition() <= 0) {
            zeroEncoder();

            // If the elevator is traveling downwards stop the belt motor and end the current command.
            if (currentPosition.getPositionTicks() <= previousPosition.getPositionTicks()) {
                stop();
            }
        }
    }

    public void setPosition(ElevatorPosition elevatorPosition) {
        if (currentPosition == elevatorPosition && atPosition()) {
            return;
        }
        
        previousPosition = currentPosition;
        currentPosition = elevatorPosition;
        
        // If the arm mechanism drags down the belt due to gravity add an arbitrary feed forward constant.
        beltMotor.set(
            ControlMode.MotionMagic,
            elevatorPosition.getPositionTicks()
        );
    }

    public ElevatorPosition getPosition() {
        return currentPosition;
    }

    public ElevatorPosition getPreviouPosition() {
        return previousPosition;
    }

    public boolean atPosition() {
        return Math.abs(
            currentPosition.getPositionTicks() - beltMotor.getSelectedSensorPosition()
        ) <= Constants.Elevator.BELT_MOTOR_DEAD_ZONE_TICKS;
    }

    public boolean elevatorZeroed() {
        // Limit switch returns true by default.
        return !limitSwitch.get();
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
        beltMotor.set(TalonFXControlMode.PercentOutput, Constants.Elevator.MANUAL_UP_SPEED);
    }

    public void manualDown() {
        if (!elevatorZeroed()) {
            beltMotor.set(TalonFXControlMode.PercentOutput, Constants.Elevator.MANUAL_DOWN_SPEED);
        } else {
            stop();
        }
    }

    public void stop() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, Constants.Elevator.FEED_FORWARD);
    }

    public static enum ElevatorPosition {
        STARTING(0),
        FLOOR_CUBE(7200),
        FLOOR_CONE(20000),
        BABY_BIRD(16000),
        MID_SCORE(92000),
        TOP_SCORE(125000);

        private final int positionTicks;

        private ElevatorPosition(int positionTicks) {
            this.positionTicks = positionTicks;
        }

        public int getPositionTicks() {
            return positionTicks;
        }
    }
}
