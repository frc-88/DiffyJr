package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.team88.chassis.ChassisInterface;
import frc.team88.diffswerve.DiffSwerveChassis;
import frc.team88.diffswerve.DiffSwerveModule;
import frc.team88.tunnel.PacketResult;
import frc.team88.tunnel.ROSInterface;
import frc.team88.tunnel.TunnelClient;
import frc.team88.tunnel.TunnelServer;
import frc.team88.waypoints.Waypoint;
import frc.team88.waypoints.WaypointsPlan;

public class DiffyTunnelInterface extends ROSInterface {
    private WaypointsPlan plan;
    private DiffSwerveChassis swerve;

    public DiffyTunnelInterface(DiffSwerveChassis swerve) {
        super((ChassisInterface)swerve);
        this.swerve = swerve;

        plan = new WaypointsPlan(this);
        plan.addWaypoint(new Waypoint("goal1"));
        plan.addWaypoint(new Waypoint("goal2").makeContinuous(true));
        plan.addWaypoint(new Waypoint("goal3").makeContinuous(true));
        plan.addWaypoint(new Waypoint("goal4"));
        plan.addWaypoint(new Waypoint("goal5"));
        plan.addWaypoint(new Waypoint("goal1").makeIgnoreOrientation(false));
    }

    @Override
    public void packetCallback(TunnelClient tunnel, PacketResult result) {
        super.packetCallback(tunnel, result);

        String category = result.getCategory();

        if (category.equals("general")) {
            int general_cmd = (int) result.get(0);
            switch (general_cmd) {
                case 1:
                    plan.sendWaypoints();
                    break;
                case 2:
                    this.swerve.resetImu();
                default:
                    break;
            }
        }
    }

    @Override
    public void update() {
        super.update();
        TunnelServer.writePacket("imu",
            this.swerve.imu.getYaw(), this.swerve.imu.getYawRate(),
            this.swerve.imu.getAccelX(), this.swerve.imu.getAccelY()
        );
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            TunnelServer.writePacket("module",
                index,
                state.angle.getRadians(),
                state.speedMetersPerSecond,
                module.getLoNextVoltage(),
                module.getLoRadiansPerSecond(),
                module.getHiNextVoltage(),
                module.getHiRadiansPerSecond()
            );
        }
    }
}
