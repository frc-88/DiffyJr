package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class TestTrajectories {
    public static Trajectory simpleTest1(TrajectoryConfig config) {
        Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d());

        ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(0.5, 0.0));

        Pose2d end = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(180.0));

        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    }
}
