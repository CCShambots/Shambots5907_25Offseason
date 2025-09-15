package frc.robot.subsystems.climber;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberIOReal implements ClimberIO {
    TalonFX winchMotor = new TalonFX(MOTOR1_ID);
    TalonFX beltMotor = new TalonFX(MOTOR2_ID);

    public ClimberIOReal() {
        winchMotor.getConfigurator().apply(MOTOR_CONFIGURATION);
        winchMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(INVERT_MOTOR1));
        beltMotor.getConfigurator().apply(MOTOR_CONFIGURATION);
        beltMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(INVERT_MOTOR2));
    }

    @Override
    public void set(double speed) {
        if (speed > 0) {
            winchMotor.set(speed);
            beltMotor.set(0);
        } else {
            winchMotor.set(0);
            beltMotor.set(speed);
        }
    }

    @Override
    public void winchSet(double speed) {
        winchMotor.set(speed);
    }

    @Override
    public void stop() {
        winchMotor.stopMotor();
        beltMotor.stopMotor();
    }
}
