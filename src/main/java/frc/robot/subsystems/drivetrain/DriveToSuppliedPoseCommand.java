// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToSuppliedPoseCommand extends Command {
    SlewRateLimiter xLimiter = new SlewRateLimiter(8);
    SlewRateLimiter yLimiter = new SlewRateLimiter(8);

    StatefulDrivetrain drivetrain;
    Supplier<Pose2d> targetPoseSupplier;

    HolonomicDriveController controller;

    /** Creates a new driveToPose. */
    public DriveToSuppliedPoseCommand(StatefulDrivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        controller = drivetrain.getController();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.resetPIDControllers();

        ChassisSpeeds speeds = drivetrain.getDriveState().Speeds;
        xLimiter.reset(speeds.vxMetersPerSecond);
        yLimiter.reset(speeds.vyMetersPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d targetPose = targetPoseSupplier.get();
        var chassisSpeeds = controller.calculate(currentPose, targetPose, 0.0, targetPose.getRotation());
        double xSpeed = xLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
        double ySpeed = yLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
        drivetrain.drive(xSpeed, ySpeed, chassisSpeeds.omegaRadiansPerSecond, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
