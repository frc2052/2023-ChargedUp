package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class ElevatorSubsystem {
    private TalonFX beltMotor;

    public void elevatorPos1() {
        beltMotor.set(TalonFXControlMode.MotionMagic, );

    }
    


    public void ElevatorStop() {
        beltMotor.set(TalonFXControlMode.PercentOutput, 0);
    } 
}
