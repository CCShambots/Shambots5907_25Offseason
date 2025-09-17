package frc.robot.subsystems.elevatorMech;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.controllers.ControllerBindings;
import frc.robot.subsystems.elevatorMech.IOs.AlgaeMechIO;
import frc.robot.subsystems.elevatorMech.IOs.CoralMechIO;
import frc.robot.subsystems.elevatorMech.IOs.ElevatorIO;
import frc.robot.subsystems.elevatorMech.IOs.CoralMechIO.CoralMechInputs;

public class ElevatorMech extends StateMachine<ElevatorMech.State> {

    ElevatorIO elevatorIO;
    CoralMechIO coralMechIO;
    AlgaeMechIO algaeMechIO;

    CoralMechInputs coralMechInputs = new CoralMechInputs();

    ControllerBindings bindings;

    public ElevatorMech(ElevatorIO elevatorIO, CoralMechIO coralMechIO, AlgaeMechIO algaeMechIO, ControllerBindings bindings) {
        super("ElevatorMech", State.UNDETERMINED, State.class);
        this.elevatorIO = elevatorIO;
        this.coralMechIO = coralMechIO;
        this.algaeMechIO = algaeMechIO;
        this.bindings = bindings;
        
        registerStateTransitions();
        registerStateCommands();

        elevatorIO.resetEncoder();
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, new InstantCommand(()->{
            elevatorIO.setHeight(0);
            coralMechIO.set(0);
            algaeMechIO.set(0);
        }));

        registerStateCommand(State.L1, new InstantCommand(()->{
            elevatorIO.setHeight(0.2);
            coralMechIO.set(0);
            algaeMechIO.set(0);
        }));

        registerStateCommand(State.L2, new InstantCommand(()->{
            elevatorIO.setHeight(0.35);
            coralMechIO.set(0);
            algaeMechIO.set(0);
        }));

        registerStateCommand(State.L3, new InstantCommand(()->{
            elevatorIO.setHeight(0.75);
            coralMechIO.set(0);
            algaeMechIO.set(0);
        }));

        registerStateCommand(State.INTAKE, new SequentialCommandGroup(
            new InstantCommand(()->{
                elevatorIO.setHeight(0.15);
                coralMechIO.set(0.5);
                algaeMechIO.set(0);
            }),
            new WaitUntilCommand(()->coralMechInputs.proxTriggered),
            new WaitCommand(0.15),
            new InstantCommand(()->{
                coralMechIO.set(0);
            }),
            transitionCommand(State.IDLE)
        ));

        registerStateCommand(State.ALGAE_L2, new SequentialCommandGroup(
            new InstantCommand(()->{
                elevatorIO.setHeight(0.6);
                coralMechIO.set(0);
                algaeMechIO.set(0.5);
            }),
            new WaitUntilCommand(()->bindings.confirm()),
            new InstantCommand(()->{
                algaeMechIO.set(0);
            }),
            new WaitUntilCommand(()->bindings.confirm()),
            transitionCommand(State.IDLE)
        ));

        registerStateCommand(State.ALGAE_L3, new SequentialCommandGroup(
            new InstantCommand(()->{
                elevatorIO.setHeight(0.9);
                coralMechIO.set(0);
                algaeMechIO.set(0.5);
            }),
            new WaitUntilCommand(()->bindings.confirm()),
            new InstantCommand(()->{
                algaeMechIO.set(0);
            }),
            new WaitUntilCommand(()->bindings.confirm()),
            transitionCommand(State.IDLE)
        ));

        registerStateCommand(State.PROCESSOR, new SequentialCommandGroup(
            new InstantCommand(()->{
                elevatorIO.setHeight(0.1);
                coralMechIO.set(0);
            }),
            new WaitUntilCommand(()->bindings.confirm()),
            new InstantCommand(()->{
                algaeMechIO.set(-1);
            }),
            new WaitUntilCommand(()->bindings.confirm()),
            transitionCommand(State.IDLE)
        ));

        registerStateCommand(State.AUTO_SCORE, new SequentialCommandGroup(
            new InstantCommand(()->coralMechIO.set(1)),
            new WaitCommand(0.2),
            transitionCommand(State.INTAKE)
            ));
    }

    private void registerStateTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.L1);
        addOmniTransition(State.L2);
        addOmniTransition(State.L3);
        addOmniTransition(State.INTAKE);
        addOmniTransition(State.ALGAE_L2);
        addOmniTransition(State.ALGAE_L3);
        addOmniTransition(State.PROCESSOR);
        addTransition(State.L1, State.AUTO_SCORE);
        addTransition(State.L2, State.AUTO_SCORE);
        addTransition(State.L3, State.AUTO_SCORE);
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    @Override
    protected void update() {
        coralMechIO.updateInputs(coralMechInputs);
    }
    
    public enum State {
        UNDETERMINED,
        IDLE,
        L1,
        L2,
        L3,
        INTAKE,
        ALGAE_L2,
        ALGAE_L3,
        PROCESSOR,
        AUTO_SCORE
    }
}
