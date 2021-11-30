package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffyTunnelInterface;
import frc.robot.util.GoalStatus;

public class SendCoprocessorGoal extends CommandBase {
    private final DiffyTunnelInterface tunnel_interface;
    private String waypoint_name = null;
    private Pose2d waypoint_pose = null;
    private boolean is_finished = false;
    
    private SendCoprocessorGoal(DiffyTunnelInterface tunnel_interface) {
        this.tunnel_interface = tunnel_interface;
    }

    public SendCoprocessorGoal(String waypoint_name, DiffyTunnelInterface tunnel_interface) {
        this(tunnel_interface);
        this.waypoint_name = waypoint_name;
    }
    public SendCoprocessorGoal(Pose2d waypoint_pose, DiffyTunnelInterface tunnel_interface) {
        this(tunnel_interface);
        this.waypoint_pose = waypoint_pose;
    }

    @Override
    public void initialize() {
        if (!Objects.isNull(waypoint_name)) {
            System.out.println("Sending waypoint name: " + waypoint_name);
            tunnel_interface.sendGoal(waypoint_name);
        }
        else if (!Objects.isNull(waypoint_pose)) {
            System.out.println("Sending waypoint pose: " + waypoint_pose);
            tunnel_interface.sendGoal(waypoint_pose);
        }
        else {
            System.out.println("Neither pose or name were specified for this command. Exiting command.");
            is_finished = true;
            return;
        }

        System.out.println("Starting SendCoprocessorPlan command");
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SendCoprocessorPlan command finished");
        if (interrupted) {
            System.out.println("SendCoprocessorPlan was interrupted. Cancelling goal.");
            tunnel_interface.cancelGoal();
        }
    }
}
