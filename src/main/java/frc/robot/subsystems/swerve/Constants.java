package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import java.util.Arrays;
import java.util.List;

public class Constants {

    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.00001;

    public static class DriveTrain {
        public static Pose2d robotPose;
        public static final double WIDTH = 0.154305;
        public static final double LENGTH = 0.154305;
        public static final double FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(0.0);
        public static final double FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(0.0);
        public static final double BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(0.0);
        public static final double BACK_LEFT_ENCODER_OFFSET = Math.toRadians(0.0);
        public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);
        public static final Translation2d BACK_LEFT_POSITION = new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);

        public static final double DEADBAND = 0.2;
        public static final double RACE_WHEEL_DEADBAND = 0.1;
        public static final double WHEEL_SPEED = 2; //Race Wheel turning speed
        public static double MAX_MPS = 4.5; //Speed of the swerves

        public static double SENSITIVITY_VX = 0.9;
        public static double SENSITIVITY_VY = 0.9;

        public static final double VISION_kP = 0.2;
        public static final double VISION_kI = 0.0;
        public static final double VISION_kD = 0.005;

        //        public static final double MAX_MPS = 1.0;
        public static final double MAX_ANG_VEL = Math.PI * 2.0;
        public static final double MAX_MPSS = 0.5; // accel

        public static final double ANGLE_kP = 3.5;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.0;
        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.5;
        public static final double PROFILE_CONSTRAINT_VEL = 3.0 * Math.PI;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI;
    }

    public static class DifferentialSwerveModule {

        // update rate of our modules 5ms.
        public static final double kDt = 0.005;

        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380);
        public static final int TIMEOUT = 500;
        public static final double GEAR_M11 = 5.0/72.0;
        public static final double GEAR_M12 = 7.0/72.0;
        public static final double GEAR_M21 = 1.0/24.0;
        public static final double GEAR_M22 = -1.0/24.0;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.04445; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;
        public static final double FEED_FORWARD = VOLTAGE / FALCON_FREE_SPEED;

        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final double CURRENT_LIMIT = 30.0;
        public static final double CURRENT_THRESHOLD = 30.0;
        public static final double CURRENT_TRIGGER_TIME = 0.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.005;
        public static final double INERTIA_STEER = 0.004;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
        public static final double Q_AZIMUTH = 0.08; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 5; // radians per sec
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our senlrs.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty so we increase
        // the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;
    }

    public static class Spindexer {

        public static boolean SPINDEXER_INVERTED = false;
        public static boolean FEEDER_INVERTED = false;
        public static final double SPINDEXER_IDLE_SPEED = -0.25;
        public static final double SHIMMY = 0.25;
        public static final double FEEDER_IDLE_SPEED = -0.25;
        public static final double FEEDER_SHIMMY = 0.25;
        public static final double SPINDEXER_SPEED = 0.75; // was 0.75 but I'm insane sooo...
        public static final double FEEDER_SPEED = 1.0;
    }

    public static class Field {
        /** Y ^ | | o - - -> X */
        public static final double FULL_FIELD_X = 16.0;

        public static final double HALF_FIELD_X = FULL_FIELD_X / 2.0;
        public static final double FULL_FIELD_Y = 8.21055;
        public static final double TARGET_LINE_Y = 2.404364;
        public static final double START_LINE_X = FULL_FIELD_X - 3.048;
        public static final double MID_TRENCH_Y = TARGET_LINE_Y - Units.inchesToMeters(66.91);

        public static final Pose2d TARGET_POSITION =
                new Pose2d(FULL_FIELD_X, TARGET_LINE_Y, new Rotation2d(0));
        public static final Pose2d EIGHT_BALL_TRENCH_STARTING_POSITION =
                new Pose2d(12.8, 5.8, new Rotation2d(0));
    }

    public static class Maverick{
        public static short numberOfWaypoints = 4; 
        public static double[] waypointsX = {12.10, 0.0, 0.0, 0.0};
        public static double[] waypointsY = {3.67, 0.0, 0.0, 0.0};
        public static double[] rotations = {2.0, 0.0, 0.0, 0.0};
        public static double[] tolerences = {0.0, 0.0, 0.0, 0.0};
        public static double[] speeds = {3.5, 3.5, 3.5, 3.5, 3.5};
        public static boolean[] afterburner = {false, false, false, false};
    }

    public static class AutoPath {
        public static class MoveAndGo {
            //            public static final List<Pose2d> waypoints =
            //                    Arrays.asList(
            //                            new Pose2d()
            //                    )
        }

        public static class EightBallAuto {
            public static final List<Pose2d> waypoints =
                    Arrays.asList(
                            new Pose2d(13, 2.34, Rotation2d.fromDegrees(180)),
                            new Pose2d(12, 2.34, Rotation2d.fromDegrees(180)));
        }

        public static class StealBallAuto {
            public static final List<Pose2d> stealInitial =
                    Arrays.asList(
                            new Pose2d(12.8, 5.8, Rotation2d.fromDegrees(130)),
                            new Pose2d(9.95, 7.5, Rotation2d.fromDegrees(180)));
            public static final List<Pose2d> stealPartTwo =
                    Arrays.asList(
                            new Pose2d(9.95, 7.5, Rotation2d.fromDegrees(-70)),
                            new Pose2d(10.5, 4.07, Rotation2d.fromDegrees(-100)));
            public static final List<Pose2d> stealPartThree =
                    Arrays.asList(
                            new Pose2d(10.5, 4.07, Rotation2d.fromDegrees(-100)),
                            new Pose2d(9.08, 3.4, Rotation2d.fromDegrees(-100)));
            public static final List<Pose2d> stealEnd =
                    Arrays.asList(
                            new Pose2d(8.9, 3.6, Rotation2d.fromDegrees(45)),
                            new Pose2d(11.2, 4.16, Rotation2d.fromDegrees(-10)),
                            new Pose2d(12.5, 3.15, Rotation2d.fromDegrees(-10)));
        }
    }
}
