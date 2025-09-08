package frc.robot.subsystems.elevatorMech.IOs;

public interface ElevatorIO {

    public static class ElevatorInputs {
        public double positionMeters = 0.0;
    }

    public void setHeight(double heightMeters);

    public void resetEncoder();

    public void updateInputs(ElevatorInputs inputs);
}
