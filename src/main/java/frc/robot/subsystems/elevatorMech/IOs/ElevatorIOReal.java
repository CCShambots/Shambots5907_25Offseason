package frc.robot.subsystems.elevatorMech.IOs;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.ElevatorMech.Elevator.*;

public class ElevatorIOReal implements ElevatorIO {
    TalonFX motor1 = new TalonFX(MOTOR1_ID);
    TalonFX motor2 = new TalonFX(MOTOR2_ID);
    MotionMagicVoltage request = new MotionMagicVoltage(0);

    public ElevatorIOReal() {
        motor1.getConfigurator().apply(MOTOR_CONFIGURATION);
        motor2.getConfigurator().apply(MOTOR_CONFIGURATION);
        motor1.getConfigurator().apply(new MotorOutputConfigs().withInverted(INVERT_MOTOR1).withNeutralMode(NeutralModeValue.Brake));
        motor2.getConfigurator().apply(new MotorOutputConfigs().withInverted(INVERT_MOTOR2).withNeutralMode(NeutralModeValue.Brake));
        motor2.setControl(new Follower(MOTOR1_ID, true));
    }

    @Override
    public void setHeight(double heightMeters) {
        System.out.println(heightMeters);
        motor1.setControl(request.withPosition(heightMeters / METERS_PER_ROTATION));
    }

    @Override
    public void resetEncoder() {
        motor1.setPosition(0);
        motor2.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.positionMeters = motor1.getPosition().getValueAsDouble() * METERS_PER_ROTATION;
    }
}
