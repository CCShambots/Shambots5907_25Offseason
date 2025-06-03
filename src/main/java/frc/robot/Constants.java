package frc.robot;

import java.util.List;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.vision.DefaultPostProcessor;
import frc.robot.subsystems.vision.DefaultPreProcessor;
import frc.robot.subsystems.vision.Vision.PVCamera;

public class Constants {
    public static class Vision {
        public static Double AMBIGUITY_THRESHHOLD = 0.4;

        public static final AprilTagFieldLayout FIELD_LAYOUT;
        static {
        try {
            FIELD_LAYOUT =
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
        } catch (Exception e) {
            throw new RuntimeException("Could not load AprilTag field layout");
        }
        }

        public static final Pose3d FRONT_LEFT_CAM_POSE =
            new Pose3d(
                Units.inchesToMeters(11.0),   //11.0
                Units.inchesToMeters(11.0),   //11.0
                Units.inchesToMeters(8.5),     //8.5
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(20)));

        public static final Pose3d FRONT_RIGHT_CAM_POSE =
            new Pose3d(
                Units.inchesToMeters(11.0),   //11.0
                Units.inchesToMeters(-11.0),         //-11.0
                Units.inchesToMeters(8.5),    //8.5
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(-70)));

        public static final Pose3d BACK_LEFT_CAM_POSE =
            new Pose3d(
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(11.0),
                Units.inchesToMeters(8.5),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(180+40)));
                

        public static final Pose3d BACK_RIGHT_CAM_POSE =
            new Pose3d(
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(8.5),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(180-30)));
        
        public static final double FRONT_LEFT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);
        public static final double FRONT_RIGHT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);
        public static final double BACK_LEFT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);
        public static final double BACK_RIGHT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);

        public static final int AMBIGUITY_AVG_LENGTH = 100;
        public static final double DISTANCE_SCALAR = 2;
        
        public static PVCamera FRONT_LEFT_CAM =
        new PVCamera(
            "pv_FrontLeft", 
            FRONT_LEFT_CAM_POSE,
            new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
            new DefaultPostProcessor(FIELD_LAYOUT, FRONT_LEFT_CAM_TRUST_CUTOFF)
        );

        public static PVCamera FRONT_RIGHT_CAM =
        new PVCamera(
            "pv_FrontRight", 
            FRONT_RIGHT_CAM_POSE,
            new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
            new DefaultPostProcessor(FIELD_LAYOUT, FRONT_RIGHT_CAM_TRUST_CUTOFF)
        );
    
        public static PVCamera BACK_LEFT_CAM =
        new PVCamera(
            "pv_BackLeft", 
            BACK_LEFT_CAM_POSE,
            new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
            new DefaultPostProcessor(FIELD_LAYOUT, BACK_LEFT_CAM_TRUST_CUTOFF)
        );

        public static PVCamera BACK_RIGHT_CAM =
        new PVCamera(
            "pv_BackRight", 
            BACK_RIGHT_CAM_POSE,
            new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
            new DefaultPostProcessor(FIELD_LAYOUT, BACK_RIGHT_CAM_TRUST_CUTOFF)
        );

        public static final List<PVCamera> CAMERAS = List.of(
            FRONT_LEFT_CAM,
            FRONT_RIGHT_CAM,
            BACK_LEFT_CAM,
            BACK_RIGHT_CAM
        );
    }

    public static class Autonomous {

        public static final AutoCommand[] AUTO_PATHS = new AutoCommand[] {
            
        };

        public static RobotConfig CONFIG;

        static {
            try {
                CONFIG = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                throw new RuntimeException("Could not load robot config", e);
            }
        }
    }
}
