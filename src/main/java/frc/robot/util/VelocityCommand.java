package frc.robot.util;

public class VelocityCommand {
    public double vx = 0.0; 
    public double vy = 0.0; 
    public double vt = 0.0; 
    public boolean fieldRelative = false;

    public VelocityCommand() {

    }

    public VelocityCommand(double vx, double vy, double vt, boolean fieldRelative)
    {
        this.vx = vx;
        this.vy = vy;
        this.vt = vt;
        this.fieldRelative = fieldRelative;
    }
}
