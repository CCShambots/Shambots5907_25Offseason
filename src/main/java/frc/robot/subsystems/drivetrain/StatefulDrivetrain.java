package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.SMF.StateMachine;
import frc.robot.util.AllianceManager;

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
    private Field2d field = new Field2d();

    private HolonomicDriveController alignController;

    public StatefulDrivetrain(CommandSwerveDrivetrain drivetrain, double maxSpeed, double maxAngularRate, DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier, DoubleSupplier rotSupplier) {
        super("StatefulDrivetrain", State.UNDETERMINED, State.class);
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSupplier = rotSupplier;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        alignController = new HolonomicDriveController(
            new PIDController(0.5, 0.0, 0.0),
            new PIDController(0.5, 0.0, 0.0),
            new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(maxAngularRate, 8.0))
        );
        alignController.getThetaController().enableContinuousInput(-Math.PI, Math.PI);

        registerStateCommands();
        registerStateTransitions();

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                SwerveDriveState state = drivetrain.getState();
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", ()->state.ModulePositions[0].angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", ()->state.ModuleStates[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", ()->state.ModulePositions[1].angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", ()->state.ModuleStates[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", ()->state.ModulePositions[2].angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", ()->state.ModuleStates[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", ()->state.ModulePositions[3].angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", ()->state.ModuleStates[3].speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", ()->state.Pose.getRotation().getRadians(), null);
            }
        });
    }

    public Pose2d getPose() {
        return drivetrain.getPose();
    }

    public void syncHeading() {
        double allianceHeading = AllianceManager.getAlliance()==Alliance.Red ? 180.0 : 0.0;
        Rotation2d newHeading = Rotation2d.fromDegrees(allianceHeading);
        drivetrain.setOperatorPerspectiveForward(newHeading);
    }

    public void resetHeading() {
        Pose2d pose = drivetrain.getPose();
        drivetrain.resetPose(new Pose2d(pose.getTranslation(), new Rotation2d()));
    }

    public void resetPose() {
        drivetrain.resetPose(new Pose2d());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        drivetrain.applyRequest(
            ()->drivetrain.generateRequest(xSpeed*maxSpeed, ySpeed*maxSpeed, rot*maxAngularRate, fieldRelative)
        ).schedule();
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    @Override
    public void update() {
        field.setRobotPose(drivetrain.getPose());
        SmartDashboard.putData("Match Field", field);
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

        registerStateCommand(State.ALIGN_REEF_L, new DriveToSuppliedPoseCommand(this, ()->Constants.Autonomous.getClosestReefPose(true, getPose())));
        registerStateCommand(State.ALIGN_REEF_R, new DriveToSuppliedPoseCommand(this, ()->Constants.Autonomous.getClosestReefPose(false, getPose())));
    }

    private void registerStateTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.TRAVERSING);
        addOmniTransition(State.X_SHAPE);
        addOmniTransition(State.ALIGN_REEF_L);
        addOmniTransition(State.ALIGN_REEF_R);
    }

    public void resetPIDControllers() {
        alignController.getXController().reset();
        alignController.getYController().reset();
        alignController.getThetaController().reset(getDriveState().Pose.getRotation().getRadians(), getDriveState().Speeds.omegaRadiansPerSecond);
    }

    public HolonomicDriveController getController() {
        return alignController;
    }

    public SwerveDriveState getDriveState() {
        return drivetrain.getState();
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        TRAVERSING,
        X_SHAPE,
        ALIGN_REEF_L,
        ALIGN_REEF_R
    }
}
