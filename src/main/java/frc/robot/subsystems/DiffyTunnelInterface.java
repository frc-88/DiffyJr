package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.DiffSwerveChassis;
import frc.team88.tunnel.PacketResult;
import frc.team88.tunnel.TunnelInterface;
import frc.team88.tunnel.TunnelServer;
import frc.team88.tunnel.TunnelClient;

public class DiffyTunnelInterface implements TunnelInterface {
    private TunnelServer server;
    private DiffSwerveChassis swerve;
    private long last_command_time = 0;
    private TunnelClient last_command_client;
    private final long ACTIVE_TIME_THRESHOLD = 1_000_000; // microseconds
    private double commandVx = 0.0;
    private double commandVy = 0.0;
    private double commandVt = 0.0;

    public DiffyTunnelInterface(DiffSwerveChassis swerve) {
        this.swerve = swerve;
    }

    @Override
    public HashMap<String, String> getCategories() {
        return new HashMap<String, String>() {
            private static final long serialVersionUID = 1L;
            {
                put("ping", "f");
                put("cmd", "fff");
            }
        };
    }

    @Override
    public void packetCallback(TunnelClient tunnel, PacketResult result) {
        String category = result.getCategory();
        if (category.equals("cmd")) {
            commandVx = (double) result.get(0);
            commandVy = (double) result.get(1);
            commandVt = (double) result.get(2);
            resetCommandTimer(tunnel);
        }
        else if (category.equals("ping")) {
            tunnel.writePacket("ping", (double) result.get(0));
        }
    }

    @Override
    public void update() {
        Pose2d pose = this.swerve.getOdometryPose();
        ChassisSpeeds velocity = this.swerve.getChassisVelocity();
        this.server.writePacket("odom",
            pose.getX(), pose.getY(), pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond
        );
    }

    @Override
    public void setTunnelServer(TunnelServer server) {
        this.server = server;
    }

    private long getTime() {
        return RobotController.getFPGATime();
    }

    private void resetCommandTimer(TunnelClient client) {
        last_command_time = getTime();
        last_command_client = client;
    }

    private boolean isLastTunnelOpen() {
        if (Objects.isNull(last_command_client)) {
            return false;
        }
        return last_command_client.isAlive() && last_command_client.isOpen();
    }

    public boolean isCommandActive() {
        return isLastTunnelOpen() && getTime() - last_command_time < ACTIVE_TIME_THRESHOLD;
    }

    public double getCommandVx() {
        return commandVx;
    }

    public double getCommandVy() {
        return commandVy;
    }

    public double getCommandVt() {
        return commandVt;
    }
}
