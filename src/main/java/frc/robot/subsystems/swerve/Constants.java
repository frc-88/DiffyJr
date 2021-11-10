package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.00001;

    public static class DriveTrain {
        public static Pose2d robotPose;
        public static final double WIDTH = 0.154305;
        public static final double LENGTH = 0.154305;
        public static final double FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(136.93);
        public static final double BACK_LEFT_ENCODER_OFFSET = Math.toRadians(-108.635);
        public static final double BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(-46.68);
        public static final double FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(-141.85);
        public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_LEFT_POSITION = new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);

        public static final double DEADBAND = 0.2;
        public static final double RACE_WHEEL_DEADBAND = 0.1;

        public static final double MAX_CHASSIS_SPEED = 4.5;  // Maximum chassis speed (m/s)
        public static final double MAX_CHASSIS_ANG_VEL = Math.toRadians(90.0);  // Maximum chassis rotational velocity (rad/s)
        public static final double MAX_CHASSIS_LINEAR_ACCEL = 0.5; // Maximum chassis linear acceleration (m/s^2)

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
        public static final double FALCON_TICKS_TO_ROTATIONS = 1.0 / 2048.0;
        public static final double AZIMUTH_TICKS_TO_ROTATIONS = 1.0 / 360.0;
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
}
