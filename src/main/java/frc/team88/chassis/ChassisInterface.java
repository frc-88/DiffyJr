package frc.team88.chassis;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public interface ChassisInterface {
    public void drive(VelocityCommand command);
    public void drive(double vx, double vy, double angularVelocity);
    public void holdDirection();
    public void resetOdom(Pose2d pose);
    public Pose2d getOdometryPose();
    public ChassisSpeeds getChassisVelocity();
}
