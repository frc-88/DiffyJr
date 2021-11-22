package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.TrajectoryBuilder.PathValidity;
import frc.robot.subsystems.swerve.DiffSwerveChassis;
import frc.robot.subsystems.swerve.DiffSwerveModule;
import frc.team88.tunnel.PacketResult;
import frc.team88.tunnel.TunnelInterface;
import frc.team88.tunnel.TunnelServer;
import frc.team88.tunnel.TunnelClient;

public class DiffyTunnelInterface implements TunnelInterface {
    private final DiffSwerveChassis swerve;
    private final TrajectoryBuilder traj_builder;

    private long last_command_time = 0;
    private TunnelClient last_command_client;
    private final long ACTIVE_TIME_THRESHOLD = 1_000_000; // microseconds
    private double commandVx = 0.0;
    private double commandVy = 0.0;
    private double commandVt = 0.0;

    private CommandBase follow_traj_command;

    public DiffyTunnelInterface(DiffSwerveChassis swerve, TrajectoryBuilder traj_builder) {
        this.swerve = swerve;
        this.traj_builder = traj_builder;
    }

    @Override
    public HashMap<String, String> getCategories() {
        return new HashMap<String, String>() {
            private static final long serialVersionUID = 1L;
            {
                put("ping", "f");
                put("cmd", "fff");
                put("reset", "fff");
                put("global", "fff");
                put("plan", "ddfff");
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
        else if (category.equals("global")) {
            double x = (double) result.get(0);
            double y = (double) result.get(1);
            double theta = (double) result.get(2);
            swerve.setGlobalPose(new Pose2d(x, y, new Rotation2d(theta)));
        }
        else if (category.equals("plan")) {
            int index = (int) result.get(0);
            int total = (int) result.get(1);
            double x = (double) result.get(2);
            double y = (double) result.get(3);
            double theta = (double) result.get(4);
            traj_builder.addWaypoint(index, total, new Pose2d(x, y, new Rotation2d(theta)));

            if (traj_builder.getPathValidity() == PathValidity.YES) {
                if (!Objects.isNull(follow_traj_command)) {
                    follow_traj_command.cancel();
                }
                System.out.println("Running new plan");
                follow_traj_command = new FollowTrajectory(this.swerve, traj_builder, traj_builder.getTrajectory(this.swerve.getTrajectoryConfig()));
                follow_traj_command.schedule();
            }
        }
        else if (category.equals("reset")) {
            double x = (double) result.get(0);
            double y = (double) result.get(1);
            double theta = (double) result.get(2);
            this.swerve.resetOdom(new Pose2d(new Translation2d(x, y), new Rotation2d(theta)));
        }
    }

    @Override
    public void update() {
        Pose2d pose = this.swerve.getOdometryPose();
        ChassisSpeeds velocity = this.swerve.getChassisVelocity();
        TunnelServer.writePacket("odom",
            pose.getX(), pose.getY(), pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond
        );
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
