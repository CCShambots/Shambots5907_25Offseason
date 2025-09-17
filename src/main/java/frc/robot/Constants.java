package frc.robot;

// import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.util.AutoCommand;
// import frc.robot.subsystems.vision.DefaultPostProcessor;
// import frc.robot.subsystems.vision.DefaultPreProcessor;
// import frc.robot.subsystems.vision.Vision.PVCamera;

public class Constants {

    public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40);

    public static class Field {
        public static final double fieldWidthMeters = 805.0 / 100.0; // 805 cm wide field
        public static final double fieldLengthMeters = 1755.0 / 100.0; // 1755 cm long field
    }

    public static class Vision {
        /*
         * public static Double AMBIGUITY_THRESHHOLD = 0.4;
         * 
         * public static final AprilTagFieldLayout FIELD_LAYOUT;
         * static {
         * try {
         * FIELD_LAYOUT = AprilTagFieldLayout
         * .loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
         * } catch (Exception e) {
         * throw new RuntimeException("Could not load AprilTag field layout");
         * }
         * }
         * 
         * public static final Pose3d FRONT_LEFT_CAM_POSE = new Pose3d(
         * Units.inchesToMeters(11.0), // 11.0
         * Units.inchesToMeters(11.0), // 11.0
         * Units.inchesToMeters(8.5), // 8.5
         * new Rotation3d(0, Math.toRadians(0), Math.toRadians(20)));
         * 
         * public static final Pose3d FRONT_RIGHT_CAM_POSE = new Pose3d(
         * Units.inchesToMeters(11.0), // 11.0
         * Units.inchesToMeters(-11.0), // -11.0
         * Units.inchesToMeters(8.5), // 8.5
         * new Rotation3d(0, Math.toRadians(0), Math.toRadians(-70)));
         * 
         * public static final Pose3d BACK_LEFT_CAM_POSE = new Pose3d(
         * Units.inchesToMeters(-11.0),
         * Units.inchesToMeters(11.0),
         * Units.inchesToMeters(8.5),
         * new Rotation3d(0, Math.toRadians(0), Math.toRadians(180 + 40)));
         * 
         * public static final Pose3d BACK_RIGHT_CAM_POSE = new Pose3d(
         * Units.inchesToMeters(-11.0),
         * Units.inchesToMeters(-11.0),
         * Units.inchesToMeters(8.5),
         * new Rotation3d(0, Math.toRadians(0), Math.toRadians(180 - 30)));
         * 
         * public static final double FRONT_LEFT_CAM_TRUST_CUTOFF =
         * Units.feetToMeters(14);
         * public static final double FRONT_RIGHT_CAM_TRUST_CUTOFF =
         * Units.feetToMeters(14);
         * public static final double BACK_LEFT_CAM_TRUST_CUTOFF =
         * Units.feetToMeters(14);
         * public static final double BACK_RIGHT_CAM_TRUST_CUTOFF =
         * Units.feetToMeters(14);
         * 
         * public static final int AMBIGUITY_AVG_LENGTH = 100;
         * public static final double DISTANCE_SCALAR = 2;
         * 
         * public static PVCamera FRONT_LEFT_CAM = new PVCamera(
         * "pv_FrontLeft",
         * FRONT_LEFT_CAM_POSE,
         * new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR,
         * AMBIGUITY_AVG_LENGTH),
         * new DefaultPostProcessor(FIELD_LAYOUT, FRONT_LEFT_CAM_TRUST_CUTOFF));
         * 
         * public static PVCamera FRONT_RIGHT_CAM = new PVCamera(
         * "pv_FrontRight",
         * FRONT_RIGHT_CAM_POSE,
         * new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR,
         * AMBIGUITY_AVG_LENGTH),
         * new DefaultPostProcessor(FIELD_LAYOUT, FRONT_RIGHT_CAM_TRUST_CUTOFF));
         * 
         * public static PVCamera BACK_LEFT_CAM = new PVCamera(
         * "pv_BackLeft",
         * BACK_LEFT_CAM_POSE,
         * new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR,
         * AMBIGUITY_AVG_LENGTH),
         * new DefaultPostProcessor(FIELD_LAYOUT, BACK_LEFT_CAM_TRUST_CUTOFF));
         * 
         * public static PVCamera BACK_RIGHT_CAM = new PVCamera(
         * "pv_BackRight",
         * BACK_RIGHT_CAM_POSE,
         * new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR,
         * AMBIGUITY_AVG_LENGTH),
         * new DefaultPostProcessor(FIELD_LAYOUT, BACK_RIGHT_CAM_TRUST_CUTOFF));
         * 
         * public static final List<PVCamera> CAMERAS = List.of(
         * FRONT_LEFT_CAM,
         * FRONT_RIGHT_CAM,
         * BACK_LEFT_CAM,
         * BACK_RIGHT_CAM);
         */
    }

    public static class Limelight {
        public static final double FORWARD_OFFSET = Units.inchesToMeters(5.67);
        public static final double LATERAL_OFFSET = Units.inchesToMeters(13.875);
        public static final double VERTICAL_OFFSET = Units.inchesToMeters(30.0);

        public static final double PITCH_OFFSET = -45.0; // degrees
        public static final double YAW_OFFSET = 0.0; // degrees
        public static final double ROLL_OFFSET = 0.0; // degrees
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

        public static final double redReefX = Units.inchesToMeters(514.13);
        public static final double redReefY = Units.inchesToMeters(158.5);

        public static final double blueReefX = Units.inchesToMeters(176.745);
        public static final double blueReefY = Units.inchesToMeters(158.5);

        public static final double bumperToRobotOrigin = Units.inchesToMeters(18.0);
        public static final double lrOffset = Units.inchesToMeters(9);

        public static final double scoringOffset = Units.inchesToMeters(6.5);
        public static final double reefCenterToWall = Units.inchesToMeters(32.0);

        public static final double[] angles = {
                0.0,
                Math.PI / 3,
                2 * Math.PI / 3,
                Math.PI,
                4 * Math.PI / 3,
                5 * Math.PI / 3
        };

        public static Pose2d getClosestReefPose(boolean leftSide, Pose2d robotPose) {
            Translation2d robotTranslation = robotPose.getTranslation();
            double realScoringOffset = leftSide ? scoringOffset : -scoringOffset;

            double x, y;
            if (new Translation2d(redReefX, redReefY).getDistance(
                    robotTranslation) < new Translation2d(blueReefX, blueReefY).getDistance(robotTranslation)) {
                x = redReefX;
                y = redReefY;
            } else {
                x = blueReefX;
                y = blueReefY;
            }

            double lastDistance = 1_000_000.0;
            Pose2d angledPose = new Pose2d();

            for (double angle : angles) {
                Pose2d testPose = new Pose2d(
                        new Translation2d(
                                x + (reefCenterToWall + bumperToRobotOrigin) * Math.cos(angle)
                                        + realScoringOffset * Math.cos(angle + Math.PI / 2.0),
                                y + (reefCenterToWall + bumperToRobotOrigin) * Math.sin(angle)
                                        + realScoringOffset * Math.sin(angle + Math.PI / 2.0)),
                        new Rotation2d(angle + Math.PI) // face reef center
                );

                double distance = testPose.getTranslation().getDistance(robotTranslation);
                if (distance < lastDistance) {
                    lastDistance = distance;
                    angledPose = testPose;
                }
            }

            Translation2d offsetVector = new Translation2d((lrOffset+realScoringOffset) * Math.cos(angledPose.getRotation().getRadians() + Math.PI / 2), (lrOffset+realScoringOffset) * Math.sin(angledPose.getRotation().getRadians() + Math.PI / 2));

            angledPose = new Pose2d(
                    angledPose.getTranslation().plus(offsetVector),
                    angledPose.getRotation());

            return angledPose;
        }
    }

    public static class Drivetrain {
        public static final double MAX_ANGULAR_RATE = 4 * Math.PI;
    }

    public static class ElevatorMech {
        public static class AlgaeMech {
            public static final double INTAKE_SPEED = 0.5;
            public static final double OUTTAKE_SPEED = -0.5;

            public static final int LEFT_MOTOR_ID = 20;
            public static final int RIGHT_MOTOR_ID = 21;

            public static final boolean INVERT_LEFT_MOTOR = false;
            public static final boolean INVERT_RIGHT_MOTOR = true;

            public static final SparkMaxConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig();
            public static final SparkMaxConfig RIGHT_MOTOR_CONFIG = new SparkMaxConfig();
            static {
                LEFT_MOTOR_CONFIG.inverted(INVERT_LEFT_MOTOR);
                LEFT_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
                LEFT_MOTOR_CONFIG.smartCurrentLimit(40);

                RIGHT_MOTOR_CONFIG.inverted(INVERT_LEFT_MOTOR);
                RIGHT_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
                RIGHT_MOTOR_CONFIG.smartCurrentLimit(40);
            }
        }

        public static class CoralMech {
            public static final double SPEED = 0.5;

            public static final int MOTOR_ID = 22;

            public static final InvertedValue INVERT_MOTOR = InvertedValue.Clockwise_Positive;

            public static final TalonFXSConfiguration MOTOR_CONFIGURATION = new TalonFXSConfiguration()
                    .withCurrentLimits(DEFAULT_CURRENT_LIMITS);

            static {
                MOTOR_CONFIGURATION.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
            }

            public static final int PROX_SENSOR_ID = 3;
        }

        public static class Elevator {
            public static final int MOTOR1_ID = 9;
            public static final int MOTOR2_ID = 10;

            public static final double MAX_HEIGHT_METERS = 1.0;
            public static final double MIN_HEIGHT_METERS = 0.0;

            public static final InvertedValue INVERT_MOTOR1 = InvertedValue.Clockwise_Positive;
            public static final InvertedValue INVERT_MOTOR2 = InvertedValue.CounterClockwise_Positive;

            public static final double METERS_PER_ROTATION = Units.inchesToMeters(1.87) * Math.PI / (15.0 * 1) * 2;

            public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
                    .withSlot0(new Slot0Configs()
                            .withKS(0.0)
                            .withKV(0.1)
                            .withKG(0.0)
                            .withKP(0.5)
                            .withKI(0.0)
                            .withKD(0.0))
                    .withMotionMagic(new MotionMagicConfigs()
                            .withMotionMagicAcceleration(40.0)
                            .withMotionMagicCruiseVelocity(40.0)
                            .withMotionMagicJerk(800.0))
                    .withCurrentLimits(DEFAULT_CURRENT_LIMITS);
        }
    }

    public static class Climber {
        public static final int MOTOR1_ID = 40;
        public static final int MOTOR2_ID = 41;
        public static final InvertedValue INVERT_MOTOR1 = InvertedValue.Clockwise_Positive;
        public static final InvertedValue INVERT_MOTOR2 = InvertedValue.CounterClockwise_Positive;

        public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
                .withCurrentLimits(DEFAULT_CURRENT_LIMITS);
    }
}
