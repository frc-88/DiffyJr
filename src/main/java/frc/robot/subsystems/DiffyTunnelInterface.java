package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import frc.team88.swerve.SwerveController;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.tunnel.PacketResult;
import frc.team88.tunnel.TunnelInterface;
import frc.team88.tunnel.TunnelServer;
import frc.team88.tunnel.TunnelClient;

public class DiffyTunnelInterface implements TunnelInterface {
    private SwerveController swerve;
    private long last_command_time = 0;
    private TunnelClient last_command_client;
    private final long ACTIVE_TIME_THRESHOLD = 1_000_000; // microseconds
    private VelocityState swerveCommand = new VelocityState(0.0, 0.0, 0.0, false);

    public DiffyTunnelInterface(SwerveController swerve) {
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
            swerveCommand = new VelocityState(
                (double) result.get(0),
                (double) result.get(1),
                (double) result.get(2),
                false
            );
            resetCommandTimer(tunnel);
        }
        else if (category.equals("ping")) {
            tunnel.writePacket("ping", (double) result.get(0));
        }
    }

    @Override
    public void update() {
        OdomState odom = this.swerve.getOdometry();
        TunnelServer.instance.writePacket("odom",
            odom.getXPosition(), odom.getYPosition(), odom.getTheta(),
            odom.getXVelocity(), odom.getYVelocity(), odom.getThetaVelocity()
        );
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

    public VelocityState getCommand() {
        return swerveCommand;
    }
}
