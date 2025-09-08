package frc.robot;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.util.AutoCommand;
import frc.robot.subsystems.vision.DefaultPostProcessor;
import frc.robot.subsystems.vision.DefaultPreProcessor;
import frc.robot.subsystems.vision.Vision.PVCamera;

public class Constants {

    public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMITS = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40);

    public static class Field {
        public static final double fieldWidthMeters = 805.0 / 100.0; // 805 cm wide field
        public static final double fieldLengthMeters = 1755.0 / 100.0; // 1755 cm long field
    }

    public static class Vision {
        public static Double AMBIGUITY_THRESHHOLD = 0.4;

        public static final AprilTagFieldLayout FIELD_LAYOUT;
        static {
            try {
                FIELD_LAYOUT = AprilTagFieldLayout
                        .loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
            } catch (Exception e) {
                throw new RuntimeException("Could not load AprilTag field layout");
            }
        }

        public static final Pose3d FRONT_LEFT_CAM_POSE = new Pose3d(
                Units.inchesToMeters(11.0), // 11.0
                Units.inchesToMeters(11.0), // 11.0
                Units.inchesToMeters(8.5), // 8.5
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(20)));

        public static final Pose3d FRONT_RIGHT_CAM_POSE = new Pose3d(
                Units.inchesToMeters(11.0), // 11.0
                Units.inchesToMeters(-11.0), // -11.0
                Units.inchesToMeters(8.5), // 8.5
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(-70)));

        public static final Pose3d BACK_LEFT_CAM_POSE = new Pose3d(
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(11.0),
                Units.inchesToMeters(8.5),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(180 + 40)));

        public static final Pose3d BACK_RIGHT_CAM_POSE = new Pose3d(
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(-11.0),
                Units.inchesToMeters(8.5),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(180 - 30)));

        public static final double FRONT_LEFT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);
        public static final double FRONT_RIGHT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);
        public static final double BACK_LEFT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);
        public static final double BACK_RIGHT_CAM_TRUST_CUTOFF = Units.feetToMeters(14);

        public static final int AMBIGUITY_AVG_LENGTH = 100;
        public static final double DISTANCE_SCALAR = 2;

        public static PVCamera FRONT_LEFT_CAM = new PVCamera(
                "pv_FrontLeft",
                FRONT_LEFT_CAM_POSE,
                new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
                new DefaultPostProcessor(FIELD_LAYOUT, FRONT_LEFT_CAM_TRUST_CUTOFF));

        public static PVCamera FRONT_RIGHT_CAM = new PVCamera(
                "pv_FrontRight",
                FRONT_RIGHT_CAM_POSE,
                new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
                new DefaultPostProcessor(FIELD_LAYOUT, FRONT_RIGHT_CAM_TRUST_CUTOFF));

        public static PVCamera BACK_LEFT_CAM = new PVCamera(
                "pv_BackLeft",
                BACK_LEFT_CAM_POSE,
                new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
                new DefaultPostProcessor(FIELD_LAYOUT, BACK_LEFT_CAM_TRUST_CUTOFF));

        public static PVCamera BACK_RIGHT_CAM = new PVCamera(
                "pv_BackRight",
                BACK_RIGHT_CAM_POSE,
                new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
                new DefaultPostProcessor(FIELD_LAYOUT, BACK_RIGHT_CAM_TRUST_CUTOFF));

        public static final List<PVCamera> CAMERAS = List.of(
                FRONT_LEFT_CAM,
                FRONT_RIGHT_CAM,
                BACK_LEFT_CAM,
                BACK_RIGHT_CAM);
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

            public static final int MOTOR_ID = 30;

            public static final InvertedValue INVERT_MOTOR = InvertedValue.Clockwise_Positive;

            public static final TalonFXSConfiguration MOTOR_CONFIGURATION = new TalonFXSConfiguration()
                .withCurrentLimits(DEFAULT_CURRENT_LIMITS);

            public static final int PROX_SENSOR_ID = 3;
        }

        public static class Elevator {
            public static final int MOTOR1_ID = 10;
            public static final int MOTOR2_ID = 11;

            public static final double MAX_HEIGHT_METERS = 1.0;
            public static final double MIN_HEIGHT_METERS = 0.0;

            public static final InvertedValue INVERT_MOTOR1 = InvertedValue.Clockwise_Positive;
            public static final InvertedValue INVERT_MOTOR2 = InvertedValue.CounterClockwise_Positive;

            public static final double METERS_PER_ROTATION =
                Units.inchesToMeters(1.87) * Math.PI / (15.0 * 3.0);

            public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(0.0)
                .withKV(0.0)
                .withKG(0.0)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(40.0)
                .withMotionMagicCruiseVelocity(40.0)
                .withMotionMagicJerk(800.0)
            )
            .withCurrentLimits(DEFAULT_CURRENT_LIMITS);
        }
    }

    public static class Climber {
        public static final int MOTOR_ID = 40;
        public static final InvertedValue INVERT_MOTOR = InvertedValue.Clockwise_Positive;

        public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
            .withCurrentLimits(DEFAULT_CURRENT_LIMITS);
    }
}
