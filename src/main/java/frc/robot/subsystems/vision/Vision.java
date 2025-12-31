package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class Vision {
    private final AprilTagCamera[] pvCams;
    private Field2d field = new Field2d();

    private final AprilTagFieldLayout fieldLayout;

    private Consumer<VisionEstimate[]> listeners = (estimates) -> {
    };

    private Notification connectNotification(String camName) {
        return new Notification(
                NotificationLevel.INFO,
                "Camera Connected",
                "Camera " + camName + " is now connected.");
    }

    private Notification disconnectNotification(String camName) {
        return new Notification(
                NotificationLevel.WARNING,
                "Camera Disconnected",
                "Camera " + camName + " is now disconnected.");
    }

    /**
     * Vision subsystem for handling multiple cameras and estimating poses using
     * AprilTags through PhotonVision.
     * 
     * @param camSettings List of camera settings to use for pose estimation.
     * @param fieldLayout The AprilTag field layout to use for pose estimation.
     */
    public Vision(List<PVCamera> camSettings, AprilTagFieldLayout fieldLayout) {
        this.fieldLayout = fieldLayout;
        pvCams = camSettings.stream()
                .map(
                        (settings) -> {
                            return new AprilTagCamera(settings);
                        })
                .toArray(AprilTagCamera[]::new);
    }

    /**
     * Returns the individual estimates from each cameraserver.java
     * <p>
     * Each camera will return an Optional<VisionEstimate> which contains the
     * estimated pose, standard deviations, and timestamp.
     * </p>
     * 
     * @return An array of Optional<VisionEstimate> objects, each representing the
     *         estimate from a camera.
     */
    public Optional<VisionEstimate>[] getEstimates() {
        @SuppressWarnings("unchecked")
        Optional<VisionEstimate>[] estimates = Arrays.stream(pvCams)
                .map((cam) -> cam.getEstimate(cam.lastPose))
                .toArray(size -> (Optional<VisionEstimate>[]) new Optional[size]);

        updateField2d(estimates);

        return estimates;
    }

    /**
     * Returns the estimated poses from all cameras.
     * <p>
     * This will filter out any empty estimates and return an array of
     * VisionEstimate objects.
     * </p>
     * 
     * @return An array of VisionEstimate objects containing the estimated poses.
     */
    public VisionEstimate[] getEstimatedPoses() {
        Optional<VisionEstimate>[] estimates = getEstimates();

        // Filter out empty estimates
        return Arrays.stream(estimates)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .toArray(VisionEstimate[]::new);
    }

    // Updates the internal field for visualization in the Dashboard.
    private void updateField2d(Optional<VisionEstimate>[] estimates) {
        var obj = field.getObject("Cam Estimations");
        var poses = obj.getPoses();
        obj.setPoses(Arrays.stream(estimates).map((e) -> e.isPresent()
                ? e.get().estimatedPose()
                : poses.size() > Arrays.asList(estimates).indexOf(e)
                        ? poses.get(Arrays.asList(estimates).indexOf(e))
                        : new Pose2d())
                .toList());
        SmartDashboard.putData("Estimates Field", field);
    }

    /**
     * Updates the listeners with the latest estimated poses.
     * <p>
     * This will call all registered listeners with the latest estimates.
     * </p>
     */
    public void updateListeners() {
        VisionEstimate[] estimates = getEstimatedPoses();

        if (listeners != null) {
            listeners.accept(estimates);
        } else {
            System.out.println("No listeners registered for Vision subsystem.");
        }
    }

    /**
     * Adds a listener to the Vision subsystem.
     * <p>
     * This listener will be called with the latest estimated poses whenever they
     * are updated.
     * </p>
     * 
     * @param listener The listener to add, which takes an array of VisionEstimate
     *                 objects.
     */
    public void addListener(Consumer<VisionEstimate[]> listener) {
        listeners = listeners.andThen(listener);
    }

    /**
     * Represents a camera configuration for the Vision subsystem.
     * 
     * @param name     The name of the camera.
     * @param pose     The pose of the camera in the field.
     * @param pre      The preprocessor to apply to the camera's results.
     * @param post     The postprocessor to apply to the camera's results.
     * @param strategy The pose estimation strategy to use for the camera.
     */
    public record PVCamera(
            String name,
            Pose3d pose,
            Preprocessor pre,
            Postprocessor post,
            PoseStrategy strategy) {
        /**
         * Represents a camera configuration for the Vision subsystem.
         * <p>
         * This constructor uses the default pose estimation strategy of
         * MULTI_TAG_PNP_ON_COPROCESSOR.
         * </p>
         * 
         * @param name The name of the camera.
         * @param pose The pose of the camera in the field.
         * @param pre  The preprocessor to apply to the camera's results.
         * @param post The postprocessor to apply to the camera's results.
         */
        public PVCamera(String name, Pose3d pose, Preprocessor pre, Postprocessor post) {
            this(name, pose, pre, post, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        }
    }

    // pipeline-specific stuff
    public class AprilTagCamera {
        PhotonPoseEstimator photonPoseEstimator;
        PhotonCamera camera;
        PVCamera cam;
        Pose2d lastPose = new Pose2d();

        Boolean lastStatus = false;

        /**
         * Creates an AprilTagCamera instance for the Vision subsystem.
         * 
         * @param cam The camera settings to use for pose estimation.
         */
        public AprilTagCamera(PVCamera cam) {
            this.cam = cam;
            PhotonCamera camera = new PhotonCamera(cam.name);
            this.camera = camera;
            this.photonPoseEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    cam.strategy,
                    new Transform3d(new Pose3d(), cam.pose));
        }

        /**
         * Creates an AprilTagCamera instance for the Vision subsystem with a specific
         * pose estimation strategy.
         * 
         * @param cam         The camera settings to use for pose estimation.
         * @param fieldLayout The AprilTag field layout to use for pose estimation.
         */
        public AprilTagCamera(PVCamera cam, AprilTagFieldLayout fieldLayout) {
            this.cam = cam;
            PhotonCamera camera = new PhotonCamera(cam.name);
            this.camera = camera;
            this.photonPoseEstimator = new PhotonPoseEstimator(
                    fieldLayout,
                    cam.strategy,
                    new Transform3d(new Pose3d(), cam.pose));
        }

        /**
         * Gets the latest estimate from the camera.
         * <p>
         * This method updates the camera's status and retrieves the latest pose
         * estimate based on the last known pose.
         * </p>
         * 
         * @param lastPose The last known pose of the robot.
         */
        public Optional<VisionEstimate> getEstimate(Pose2d lastPose) {
            updateStatus();

            Optional<VisionEstimate> latestEstimate = Optional.empty();

            for (var change : camera.getAllUnreadResults()) {
                photonPoseEstimator.setLastPose(lastPose);
                var est = photonPoseEstimator.update(cam.pre().preprocess(change));
                if (est.isPresent()) {
                    var estimate = cam.post().postProcess(est.get());
                    this.lastPose = estimate.estimatedPose();
                    latestEstimate = Optional.of(estimate);
                }
            }

            return latestEstimate;
        }

        // Updates the camera's connection status and sends notifications if the status
        // changes.
        private void updateStatus() {
            SmartDashboard.putBoolean("cam: " + this.camera.getName() + " good", this.camera.isConnected());
            if (this.camera.isConnected() != lastStatus) {
                lastStatus = this.camera.isConnected();
                if (lastStatus) {
                    Elastic.sendNotification(
                            connectNotification(this.camera.getName()));
                } else {
                    Elastic.sendNotification(
                            disconnectNotification(this.camera.getName()));
                }
            }
        }
    }

    /**
     * Represents a camera estimate.
     * 
     * @param estimatedPose The estimated pose of the camera.
     * @param stdDevs       The standard deviations of the estimated pose.
     * @param timestamp     The timestamp of the estimate.
     */
    public record VisionEstimate(
            Pose2d estimatedPose,
            Matrix<N3, N1> stdDevs,
            Double timestamp) {
    }
}