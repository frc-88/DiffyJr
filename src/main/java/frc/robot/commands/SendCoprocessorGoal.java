package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffyTunnelInterface;

public class SendCoprocessorGoal extends CommandBase {
    private final DiffyTunnelInterface tunnel_interface;
    private String waypoint_name = null;
    private boolean is_continuous = false;
    private boolean ignore_orientation = false;
    
    public SendCoprocessorGoal(String waypoint_name, DiffyTunnelInterface tunnel_interface, boolean is_continuous, boolean ignore_orientation) {
        this.tunnel_interface = tunnel_interface;
        this.waypoint_name = waypoint_name;
        this.is_continuous = is_continuous;
        this.ignore_orientation = ignore_orientation;
    }

    public SendCoprocessorGoal(String waypoint_name, DiffyTunnelInterface tunnel_interface, boolean is_continuous) {
        this(waypoint_name, tunnel_interface, is_continuous, true);
    }
    public SendCoprocessorGoal(String waypoint_name, DiffyTunnelInterface tunnel_interface) {
        this(waypoint_name, tunnel_interface, false, true);
    }

    @Override
    public void initialize() {
        System.out.println("Sending waypoint name: " + waypoint_name);
        tunnel_interface.sendGoal(waypoint_name, is_continuous, ignore_orientation);

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
