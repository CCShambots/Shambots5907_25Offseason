package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SMF.StateMachine;

/**
 * Implements CommandSwerveDrivetrain with the State Machine Framework.
 * <p>
 * 
 * </p>
 */
public class StatefulDrivetrain extends StateMachine<StatefulDrivetrain.State> {
    private CommandSwerveDrivetrain drivetrain;
    private DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSupplier;
    private double maxSpeed = 0.0;
    private double maxAngularRate = 0.0;

    public StatefulDrivetrain(CommandSwerveDrivetrain drivetrain, double maxSpeed, double maxAngularRate, DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier, DoubleSupplier rotSupplier) {
        super("StatefulDrivetrain", State.UNDETERMINED, State.class);
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSupplier = rotSupplier;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        registerStateCommands();
        registerStateTransitions();
    }

    private void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        drivetrain.applyRequest(
            ()->drivetrain.generateRequest(xSpeed*maxSpeed, ySpeed*maxSpeed, rot*maxAngularRate, fieldRelative)
        ).schedule();
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, new RunCommand(()->{
            drivetrain.applyRequest(()->drivetrain.generateRequest(0, 0, 0, false));
        }));
        registerStateCommand(State.TRAVERSING, new RunCommand(()->{
            drive(xSpeedSupplier.getAsDouble(), ySpeedSupplier.getAsDouble(), rotSupplier.getAsDouble(), true);
        }));
        registerStateCommand(State.X_SHAPE, new RunCommand(()->{
            drivetrain.applyRequest(()->new SwerveRequest.SwerveDriveBrake());
        }));
    }

    private void registerStateTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.TRAVERSING);
        addOmniTransition(State.X_SHAPE);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        TRAVERSING,
        X_SHAPE
    }
}
