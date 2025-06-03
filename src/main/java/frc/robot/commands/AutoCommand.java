package frc.robot.commands;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class AutoCommand extends SequentialCommandGroup {
    private Trajectory trajectory;

    /** Creates a new AutoCommand.
     * <p>Extends SequentialCommandGroup to allow for sequential execution of commands.
     * Its main purpose is to allow for trajectory display. from PathPlanner Paths</p>
     */
    public AutoCommand(String name) {
        super();
        setName(name);
        // Initialize the trajectory to an empty trajectory.
        this.trajectory = new Trajectory();
    }

    /** Adds a PathPlanner Path to the AutoCommand.
     * <p>This will add a command to follow the path, and also add the trajectory to the AutoCommand's trajectory for display purposes.</p>
     * 
     * @param pathName The name of the path file (without extension) to add
     * @return The AutoCommand instance for chaining
     */
    public AutoCommand addPath(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
            Command followPathCommand = AutoBuilder.followPath(path);
            addCommands(followPathCommand);
            addTrajectoryFromPath(path);
        } catch (FileVersionException | IOException | ParseException e) {
            DriverStation.reportError("Failed to load path: \"" + pathName + "\" in auto: \"" + this.getName() + "\"", e.getStackTrace());
        }
        return this;
    }

    /** Adds a Command to be run sequentially in the Autonomous Command
     * 
     * @param command The Command to add
     * @return The AutoCommand instance for chaining
     */
    public AutoCommand addCommand(Command command) {
        addCommands(command);
        return this;
    }

    /** Adds a WaitCommand to the AutoCommand.
     * <p>This will add a command that waits for the specified number of seconds.</p>
     * 
     * @param seconds The number of seconds to wait
     * @return The AutoCommand instance for chaining
     */
    public AutoCommand addWait(double seconds) {
        addCommands(new WaitCommand(seconds)); 
        return this;
    }

    /** Adds a trajectory to the AutoCommand's trajectory.
     * <p>Does not execute a command to follow the trajectory, do this through {@link #addCommand} instead.</p>
     * 
     * @param trajectory The trajectory to add
     * @return The AutoCommand instance for chaining
     */
    public AutoCommand addTrajectory(Trajectory trajectory) {
        this.trajectory = joinTrajectories(this.trajectory, trajectory);    // don't use concatenate bc its wierd with empty trajectories
        return this;
    }

    /** Gets the trajectory of the AutoCommand.
     * <p>This is used for visualization purposes, and should not be used to follow the trajectory in a command.</p>
     * 
     * @return The trajectory of the AutoCommand
     */
    public Trajectory getTrajectory() {
        return this.trajectory;
    }

    // Adds a trajectory from a PathPlannerPath to the AutoCommand's trajectory.
    private void addTrajectoryFromPath(PathPlannerPath path) {
        Optional<PathPlannerTrajectory> trajectory = path.getIdealTrajectory(Constants.Autonomous.CONFIG);
        if (trajectory.isPresent()) {
            PathPlannerTrajectory traj = trajectory.get();
            Trajectory wpilibTrajectory = convertTrajectory(traj);
            this.trajectory = joinTrajectories(this.trajectory, wpilibTrajectory);  // don't use concatenate bc its wierd with empty trajectories
        }
        else {
            DriverStation.reportError("PathPlanner Path \"" + path.name + "\" does not have an ideal trajectory.", new Throwable().getStackTrace());
        }
    }

    // Converts a PathPlannerTrajectory to a WPILib Trajectory for visualization purposes.
    // Note: Do NOT use this for following the trajectory in a command.
    private Trajectory convertTrajectory(PathPlannerTrajectory pathPlannerTrajectory) {
        List<PathPlannerTrajectoryState> PathPlannerStates = pathPlannerTrajectory.getStates();
        List<State> states = new java.util.ArrayList<>(PathPlannerStates.size());
        for (PathPlannerTrajectoryState state : PathPlannerStates) {
            State trajState = new State();
            trajState.timeSeconds = state.timeSeconds;
            trajState.poseMeters = state.pose;
            trajState.velocityMetersPerSecond = state.linearVelocity;
            states.add(trajState);
        }
        Trajectory trajectory = new Trajectory(states);
        return trajectory;
    }

    // join multiple trajectories bc concatenate is a little wierd with empty trajectories
    private Trajectory joinTrajectories(Trajectory... trajectories) {
        List<State> joinedStates = new java.util.ArrayList<>();
        for (Trajectory trajectory : trajectories) {
            joinedStates.addAll(trajectory.getStates());
        }
        Trajectory joinedTrajectory = new Trajectory(joinedStates);
        return joinedTrajectory;
    }

    /** Creates an AutoCommand from a PathPlanner Auto.
     * <p>The AutoCommand simply runs the given auto, but generates a Trajectory for display purposes.</p>
     * 
     * @param auto The name of the PathPlanner Auto
     * @return The created AutoCommand
     */
    public static AutoCommand fromPathPlannerAuto(String autoName) {
        AutoCommand autoCommand = new AutoCommand(autoName);
        Command auto = AutoBuilder.buildAuto(autoName);
        autoCommand.addCommand(auto);
        try {
            for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(auto.getName())) {
                autoCommand.addTrajectoryFromPath(path);
            }
            Field2d field = new Field2d();
            field.getObject("Auto Trajectory").setTrajectory(autoCommand.trajectory);
            SmartDashboard.putData("Auto Trajectory", field);
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load paths for auto: " + auto.getName(), e.getStackTrace());
        }
        return autoCommand;
    }
}
