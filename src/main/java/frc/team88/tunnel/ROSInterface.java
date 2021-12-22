package frc.team88.tunnel;

import java.util.HashMap;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

import frc.team88.chassis.ChassisInterface;
import frc.team88.chassis.VelocityCommand;
import frc.team88.gameobjects.GameObject;
import frc.team88.waypoints.GoalStatus;
import frc.team88.waypoints.Waypoint;


public class ROSInterface implements TunnelInterface {
    protected ChassisInterface chassis;
    
    private MessageTimer commandTimer = new MessageTimer(1_000_000);
    private MessageTimer globalPoseTimer = new MessageTimer(1_000_000);
    private MessageTimer goalStatusTimer = new MessageTimer(1_000_000);

    protected VelocityCommand command = new VelocityCommand();
    
    protected Pose2d globalPose = new Pose2d();
    
    private final int maxNumGameObjects = 20;
    protected final GameObject[] gameObjects = new GameObject[maxNumGameObjects];
    
    protected GoalStatus goalStatus = GoalStatus.INVALID;

    private int num_sent_goals = 0;

    public ROSInterface(ChassisInterface chassis) {
        this.chassis = chassis;
    }

    @Override
    public HashMap<String, String> getCategories() {
        return new HashMap<String, String>() {
            private static final long serialVersionUID = 1L;
            {
                put("cmd", "fffd");
                put("global", "fff");
                put("obj", "ddff");
                put("gstatus", "d");
                put("ping", "f");
                put("reset", "fff");
                put("general", "d");
            }
        };
    }

    @Override
    public void packetCallback(TunnelClient tunnel, PacketResult result) {
        String category = result.getCategory();

        if (category.equals("cmd")) {
            // Velocity commands sent by the coprocessor
            command.vx = (double) result.get(0);
            command.vy = (double) result.get(1);
            command.vt = (double) result.get(2);
            command.fieldRelative = ((int) result.get(3)) == 1;
            commandTimer.setTunnelClient(tunnel);
            commandTimer.reset();
        }
        else if (category.equals("global")) {
            // Global position as declared by the coprocessor
            globalPose = new Pose2d(
                (double) result.get(0),
                (double) result.get(1),
                new Rotation2d((double) result.get(2))
            );
            globalPoseTimer.setTunnelClient(tunnel);
            globalPoseTimer.reset();
        }
        else if (category.equals("obj")) {
            // Game objects the coprocessor sees
            int index = (int) result.get(0);
            if (index < 0 || index >= maxNumGameObjects) {
                System.out.println("Invalid game object index: " + index);
                return;
            }
            gameObjects[index] = new GameObject(
                result.getRecvTime(),
                index,
                (int) result.get(1),
                (double) result.get(2),
                (double) result.get(3)
            );
        }
        else if (category.equals("gstatus")) {
            // move_base goal status
            int status = (int) result.get(0);
            goalStatus = GoalStatus.getStatus(status);
            
            goalStatusTimer.setTunnelClient(tunnel);
            goalStatusTimer.reset();
        }
        else if (category.equals("ping")) {
            tunnel.writePacket("ping", (double) result.get(0));
        }
        else if (category.equals("reset")) {
            double x = (double) result.get(0);
            double y = (double) result.get(1);
            double theta = (double) result.get(2);
            this.chassis.resetOdom(new Pose2d(new Translation2d(x, y), new Rotation2d(theta)));
        }
    }

    @Override
    public void update() {
        Pose2d pose = this.chassis.getOdometryPose();
        ChassisSpeeds velocity = this.chassis.getChassisVelocity();
        TunnelServer.writePacket("odom",
            pose.getX(), pose.getY(), pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond
        );
    }

    /***
     * Getters for data received from coprocessor
     */

    public boolean isCommandActive() {
        return commandTimer.isActive();
    }

    public VelocityCommand getCommand() {
        return command;
    }

    public Pose2d getGlobalPose() {
        return globalPose;
    }

    public boolean isGlobalPoseActive() {
        return globalPoseTimer.isActive();
    }

    public GameObject[] getGameObjects() {
        return gameObjects;
    }

    public GoalStatus getGoalStatus() {
        return goalStatus;
    }

    public boolean isGoalStatusActive() {
        return goalStatusTimer.isActive();
    }

    /***
     * Setters for sending data to the coprocessor
     */
    
    public void sendGoal(Waypoint waypoint) {
        if (num_sent_goals == 0) {
            waypoint.is_continuous = false;
            System.out.println("First goal must be discontinuous. Setting waypoint to discontinuous");
        }
        TunnelServer.writePacket("goal", waypoint.waypoint_name, waypoint.is_continuous, waypoint.ignore_orientation, waypoint.intermediate_tolerance);
        num_sent_goals++;
    }

    public void executeGoal() {
        System.out.println("Sending execute command. Num waypoints: " + num_sent_goals);
        TunnelServer.writePacket("exec", num_sent_goals);
        num_sent_goals = 0;
    }

    public void cancelGoal() {
        TunnelServer.writePacket("cancel");
    }
    
    public void resetPlan() {
        num_sent_goals = 0;
        TunnelServer.writePacket("reset");
    }

    public void sendMatchStatus(boolean motor_enabled, boolean is_autonomous, double match_timer) {
        TunnelServer.writePacket("match", motor_enabled, is_autonomous, match_timer);
    }

    public void setPoseEstimate(Pose2d poseEstimation) {
        TunnelServer.writePacket("poseest",
            poseEstimation.getTranslation().getX(),
            poseEstimation.getTranslation().getY(),
            poseEstimation.getRotation().getRadians()
        );
    }
}
