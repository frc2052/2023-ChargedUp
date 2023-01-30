package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX beltMotor;

    public ElevatorSubsystem() {
        beltMotor = new TalonFX(Constants.Elevator.CAN_ID);
        beltMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void elevatorPos1() {
        // beltMotor.set(TalonFXControlMode.MotionMagic, );
    }

    public void elvUp() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0.1);
    }

    public void stop() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void elvDown() {
        beltMotor.set(TalonFXControlMode.PercentOutput, -0.1);
    }
}
