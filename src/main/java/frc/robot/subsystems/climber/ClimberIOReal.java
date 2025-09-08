package frc.robot.subsystems.climber;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberIOReal implements ClimberIO {
    TalonFX motor = new TalonFX(MOTOR_ID);

    public ClimberIOReal() {
        motor.getConfigurator().apply(MOTOR_CONFIGURATION);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(INVERT_MOTOR));
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
