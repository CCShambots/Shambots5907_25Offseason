package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;

public interface Postprocessor {
    Vision.VisionEstimate postProcess(EstimatedRobotPose pose);
}
