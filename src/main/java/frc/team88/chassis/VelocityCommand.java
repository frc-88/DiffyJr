package frc.team88.chassis;

public class VelocityCommand {
    public double vx = 0.0; 
    public double vy = 0.0; 
    public double vt = 0.0; 

    public VelocityCommand() {

    }

    public VelocityCommand(double vx, double vy, double vt)
    {
        this.vx = vx;
        this.vy = vy;
        this.vt = vt;
    }
}
