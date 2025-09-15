package frc.robot.controllers;

public interface ControllerBindings {
    public default double xAxis() {
        return 0;
    }

    public default double yAxis() {
        return 0;
    }

    public default double turnAxis() {
        return 0;
    }

    public default boolean syncHeading() {
        return false;
    }

    public default boolean zeroHeading() {
        return false;
    }

    public default boolean confirm() {
        return true;
    }

    public default boolean stow() {
        return false;
    }

    public default boolean l1() {
        return false;
    }

    public default boolean l2() {
        return false;
    }

    public default boolean l3() {
        return false;
    }

    public default boolean intake() {
        return false;
    }

    public default boolean expel() {
        return false;
    }

    public default boolean processor() {
        return false;
    }

    public default boolean alignLeft() {
        return false;
    }

    public default boolean alignRight() {
        return false;
    }

    public default boolean climbExtend() {
        return false;
    }

    public default boolean climbUp() {
        return false;
    }

    public default boolean climbDown() {
        return false;
    }
}
