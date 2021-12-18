package frc.team88.chassis;

public interface ChassisInterface {
    public void drive(VelocityCommand command);
    public void drive(double vx, double vy, double angularVelocity, boolean fieldRelative);
    public void holdDirection();
}
