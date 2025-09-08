package frc.robot.subsystems.elevatorMech.IOs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ElevatorMech.AlgaeMech.*;

public class AlgaeMechIOReal implements AlgaeMechIO {
    SparkMax leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
    SparkMax rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

    public AlgaeMechIOReal() {
        leftMotor.configure(LEFT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(RIGHT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void set(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }
    
}
