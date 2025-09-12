package frc.robot.subsystems.elevatorMech.IOs;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.ElevatorMech.CoralMech.*;

public class CoralMechIOReal implements CoralMechIO {
    TalonFXS motor = new TalonFXS(MOTOR_ID);
    DigitalInput proxSensor = new DigitalInput(PROX_SENSOR_ID);

    public CoralMechIOReal() {
        motor.getConfigurator().apply(MOTOR_CONFIGURATION);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(INVERT_MOTOR));
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(CoralMechInputs inputs) {
        inputs.proxTriggered = !proxSensor.get();
    }
}
