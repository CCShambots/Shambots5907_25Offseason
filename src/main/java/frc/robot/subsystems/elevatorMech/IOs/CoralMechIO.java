package frc.robot.subsystems.elevatorMech.IOs;

import org.littletonrobotics.junction.AutoLog;

public interface CoralMechIO {
    
    @AutoLog
    public static class CoralMechInputs {
        public boolean proxTriggered = false;
    }

    public void set(double speed);

    public void updateInputs(CoralMechInputs inputs);
}
