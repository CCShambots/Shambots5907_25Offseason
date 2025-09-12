package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.HID.FlightStick;

public class JonahControllerBindings implements ControllerBindings {
    private final FlightStick leftFlightStick = new FlightStick(0);
    private final FlightStick rightFlightStick = new FlightStick(1);
    private final XboxController gamepad = new XboxController(2);

    @Override
    public double xAxis() {
        return leftFlightStick.getXAxis();
    }

    @Override
    public double yAxis() {
        return -leftFlightStick.getYAxis();
    }

    @Override
    public double turnAxis() {
        return -rightFlightStick.getXAxis();
    }

    @Override
    public boolean syncHeading() {
        return rightFlightStick.getTopRight();
    }

    @Override
    public boolean zeroHeading() {
        return rightFlightStick.getTopLeft();
    }

    @Override
    public boolean confirm() {
        return gamepad.getRightBumperButton();
    }

    @Override
    public boolean stow() {
        return gamepad.getLeftBumperButton();
    }

    @Override
    public boolean l1() {
        return gamepad.getPOV()==180;
    }

    @Override
    public boolean l2() {
        return gamepad.getPOV()==90;
    }

    @Override
    public boolean l3() {
        return gamepad.getPOV()==0;
    }

    @Override
    public boolean intake() {
        return gamepad.getXButton();
    }

    @Override
    public boolean expel() {
        return gamepad.getLeftTriggerAxis()>0.5;
    }

    @Override
    public boolean processor() {
        return gamepad.getBButton();
    }

    @Override
    public boolean shift() {
        return gamepad.getRightTriggerAxis()>0.5;
    }
}
