package frc.robot.subsystems.climber;

import frc.robot.SMF.StateMachine;

public class Climber extends StateMachine<Climber.State> {
    

    private final ClimberIO io;

    public Climber(ClimberIO io) {
        super("Climber", State.UNDETERMINED, State.class);
        this.io = io;

        registerStateCommands();
        registerStateTransitions();
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, () -> io.stop());
        registerStateCommand(State.EXTEND, () -> io.winchSet(0.3));
        registerStateCommand(State.CLIMBING, () -> io.set(0.3));
        registerStateCommand(State.DESCENDING, () -> io.set(-0.3));
    }

    private void registerStateTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.EXTEND);
        addOmniTransition(State.CLIMBING);
        addOmniTransition(State.DESCENDING);
    }

    @Override
    public void determineSelf(){
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,
        IDLE,
        EXTEND,
        CLIMBING,
        DESCENDING
    }
}
