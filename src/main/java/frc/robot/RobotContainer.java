// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.commands.AutoCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoManager;

public class RobotContainer extends StateMachine<RobotContainer.State>{

  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final AutoManager autoManager;

  private final Field2d preMatchField = new Field2d();

  public RobotContainer() {
    super("RobotContainer", State.PRE_MATCH, State.class);
    drivetrain = TunerConstants.createDrivetrain();
    vision  = new Vision(
      Constants.Vision.CAMERAS,
      Constants.Vision.FIELD_LAYOUT
    );
    autoManager = new AutoManager(Constants.Autonomous.AUTO_PATHS);

    vision.addListener(drivetrain::addVisionMeasurements);

    SmartDashboard.putData("Pre-Match Field", preMatchField);

    registerStateTransitions();
    registerStateCommands();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoManager.getSelectedAuto();
  }

  @Override
  protected void determineSelf() {
    setState(State.PRE_MATCH);
  }

  private void registerStateTransitions() {
    addOmniTransition(State.DISABLED);
    addOmniTransition(State.PRE_MATCH);
    addOmniTransition(State.AUTONOMOUS);
    addOmniTransition(State.TELEOP);
    addOmniTransition(State.TEST);
  }

  private void registerStateCommands() {
    registerStateCommand(State.DISABLED, () -> new InstantCommand());
    registerStateCommand(State.PRE_MATCH, () -> new InstantCommand());
    registerStateCommand(State.AUTONOMOUS, () -> new InstantCommand());
    registerStateCommand(State.TELEOP, () -> new InstantCommand());
    registerStateCommand(State.TEST, () -> new InstantCommand());
  }

  @Override
  protected void update() {
    vision.updateListeners();
    if (getState() == State.PRE_MATCH) {
      AutoCommand auto = autoManager.getSelectedAuto();
      if (auto != null) {
        Trajectory traj = auto.getTrajectory();
        if (traj.getStates().size() > 0) {
          preMatchField.getObject("Auto Trajectory").setTrajectory(traj);
          preMatchField.getObject("StartingPose").setPose(traj.getInitialPose());
        } else {
          preMatchField.getObject("Auto Trajectory").setTrajectory(new Trajectory());
          preMatchField.getObject("StartingPose").setPose(drivetrain.getPose());
        }
      } else {
        preMatchField.getObject("Auto Trajectory").setTrajectory(new Trajectory());
        preMatchField.getObject("StartingPose").setPose(drivetrain.getPose());
      }
      preMatchField.setRobotPose(drivetrain.getPose());
    }
  }

  public enum State {
    UNDETERMINED,
    DISABLED,
    PRE_MATCH,
    AUTONOMOUS,
    TELEOP,
    TEST
  }
}
