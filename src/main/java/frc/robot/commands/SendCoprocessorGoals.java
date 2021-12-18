package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffyTunnelInterface;
import frc.robot.util.Waypoint;
import frc.robot.util.WaypointsPlan;

public class SendCoprocessorGoals extends CommandBase {
    private WaypointsPlan plan;
    
    public SendCoprocessorGoals(WaypointsPlan plan) {
        this.plan = plan;
    }

    @Override
    public void initialize() {
        System.out.println("Starting SendCoprocessorPlan command");
        this.plan.sendWaypoints();
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
            this.plan.cancelPlan();
        }
    }
}
