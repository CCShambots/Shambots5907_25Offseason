// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.util.AutoCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoManager;

public class RobotContainer extends StateMachine<RobotContainer.State> {

  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final AutoManager autoManager;

  private Joystick joystick = new Joystick(0);

  private final Field2d preMatchField = new Field2d();

  public RobotContainer() {
    super("RobotContainer", State.PRE_MATCH, State.class);
    drivetrain = TunerConstants.createDrivetrain();
    vision = new Vision(
        Constants.Vision.CAMERAS,
        Constants.Vision.FIELD_LAYOUT);
    autoManager = new AutoManager(Constants.Autonomous.AUTO_PATHS);

    vision.addListener(drivetrain::addVisionMeasurements);

    SmartDashboard.putData("Pre-Match Field", preMatchField);

    registerStateCommands();
    registerStateTransitions();

    configureBindings();
  }

  private void configureBindings() {
  }

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
    registerStateCommand(State.DISABLED, new InstantCommand());
    registerStateCommand(State.PRE_MATCH, new InstantCommand());
    registerStateCommand(State.AUTONOMOUS, new InstantCommand());
    registerStateCommand(State.TELEOP, new RunCommand(() -> {
      drivetrain.applyRequest(() -> drivetrain.generateRequest(
          joystick.getX(),
          joystick.getY(),
          joystick.getZ(),
          true)).schedule();
    }));
    registerStateCommand(State.TEST, new InstantCommand());
  }

  @Override
  protected void update() {
    preMatchField.setRobotPose(drivetrain.getPose());
    vision.updateListeners();
    if (getState() == State.PRE_MATCH) {
      handlePreMatchProcesses();
    }
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TELEOP);
  }

  @Override
  protected void onDisable() {
    requestTransition(State.DISABLED);
  }

  private void handlePreMatchProcesses() {
    AutoCommand auto = autoManager.getSelectedAuto();
    String color = "#00FF00";
    if (auto != null) {
      Trajectory traj = auto.getTrajectory();
      if (traj.getStates().size() > 0) {
        preMatchField.getObject("AutoTrajectory").setTrajectory(traj);
        preMatchField.getObject("FollowPose").setPose(followDisplayTrajectory(traj));
        preMatchField.getObject("StartingPose").setPose(traj.getInitialPose());
        Double translationError =  drivetrain.getPose().getTranslation().getDistance(traj.getInitialPose().getTranslation());
        Double rotationError = Math.abs(drivetrain.getPose().getRotation().getMeasure().magnitude()-traj.getInitialPose().getRotation().getMeasure().magnitude());
        if (translationError<0.05&&rotationError<1.0) {
          color = "#00FF00";
        }
        else if (translationError<0.2&&rotationError<5.0) {
          color = "#FFAA00";
        }
        else {
          color = "#FF0000";
        }
      } else {
        preMatchField.getObject("AutoTrajectory").setTrajectory(new Trajectory());
        preMatchField.getObject("FollowPose").setPose(drivetrain.getPose());
        preMatchField.getObject("StartingPose").setPose(drivetrain.getPose());
      }
    } else {
      preMatchField.getObject("AutoTrajectory").setTrajectory(new Trajectory());
      preMatchField.getObject("FollowPose").setPose(drivetrain.getPose());
      preMatchField.getObject("StartingPose").setPose(drivetrain.getPose());
    }
    SmartDashboard.putString("Starting Proximity", color);
  }

  private Pose2d followDisplayTrajectory(Trajectory traj) {
    Double realTime = Timer.getFPGATimestamp();
    Double totalTime = traj.getTotalTimeSeconds();
    Double deltaTime = realTime % totalTime;
    Trajectory.State sample = traj.sample(deltaTime);
    return sample.poseMeters;
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
