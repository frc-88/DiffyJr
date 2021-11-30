package frc.robot.util;

public class GameObject {
    public final int obj_class;
    public final int index;
    public final double x;
    public final double y;
    public final long recv_time;

    public GameObject(long recv_time, int index, int obj_class, double x, double y) {
        this.recv_time = recv_time;
        this.obj_class = obj_class;
        this.index = index;
        this.x = x;
        this.y = y;
    }
}
