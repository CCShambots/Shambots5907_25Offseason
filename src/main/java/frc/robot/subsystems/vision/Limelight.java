package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.Vision.VisionEstimate;

public class Limelight {
    private String name;

    public Limelight(String name) {
        this.name = "limelight";
    }

    public Double getTX() {
        return LimelightHelpers.getTX(name);
    }

    public Double getTY() {
        return LimelightHelpers.getTY(name);
    }

    public Double getTA() {
        return LimelightHelpers.getTA(name);
    }

    public void set3dOffset(Pose3d offset) {
        double forward = offset.getX();
        double left = offset.getY();
        double up = offset.getZ();
        double pitch = offset.getRotation().getY();
        double yaw = offset.getRotation().getZ();
        double roll = offset.getRotation().getX();
        LimelightHelpers.setCameraPose_RobotSpace(name, forward, left, up, roll, pitch, yaw);
    }

    public VisionEstimate getBotPose(Rotation2d robotHeading) {
        LimelightHelpers.SetRobotOrientation("limelight", robotHeading.getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 == null) {
            return null;
        }
        return new VisionEstimate(
            mt2.pose,
            VecBuilder.fill(.7,.7,9999999),
            mt2.timestampSeconds
        );
    }
}
