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
    private ElevatorPosition currentDesiredPosition;

    private DigitalInput limitSwitch;
    public ElevatorSubsystem() {
        ErrorCode error;
        
        limitSwitch = new DigitalInput(0);
        
        TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.slot0.kP = Constants.Elevator.BELT_MOTOR_P;
        steerMotorConfiguration.slot0.kI = Constants.Elevator.BELT_MOTOR_I;
        steerMotorConfiguration.slot0.kD = Constants.Elevator.BELT_MOTOR_D;
        
        steerMotorConfiguration.motionCruiseVelocity = 4.0 * Constants.Elevator.BELT_MOTOR_CRUISE_VELOCITY;
        steerMotorConfiguration.motionAcceleration = 4.0 * Constants.Elevator.BELT_MOTOR_MAX_ACCELERATION;

        beltMotor = new TalonFX(Constants.Elevator.BELT_MOTOR);
        if ((error = beltMotor.configAllSettings(steerMotorConfiguration)) != ErrorCode.OK) {
            DriverStation.reportError("Failed to configure belt motor: " + error.toString(), false);
        }
        beltMotor.setNeutralMode(NeutralMode.Brake);
        beltMotor.setInverted(true);

        // Assume the elevator will start at the lowest possible position.
        zeroEncoder();
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().putData(
            Constants.Dashboard.ELEVATOR_POSITION_KEY, 
            beltMotor.getSelectedSensorPosition()
        );
        if (limitSwitch.get()){
            stop(); 
            beltMotor.setSelectedSensorPosition(0.0);
        }
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
        return Math.abs(
            currentDesiredPosition.getPositionTicks() - beltMotor.getSelectedSensorPosition()
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
        System.out.println("manual up");
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.1);
    }

    public void manualDown() {
        beltMotor.set(TalonFXControlMode.PercentOutput, -0.05);
        System.out.println("Manual down");
    }

    public void stop() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public static enum ElevatorPosition {
        STARTING(0),
        FLOORCUBE(16478),
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
