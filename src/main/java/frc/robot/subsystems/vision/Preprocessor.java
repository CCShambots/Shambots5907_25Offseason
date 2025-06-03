package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

public interface Preprocessor {
    PhotonPipelineResult preprocess(PhotonPipelineResult in);
}
