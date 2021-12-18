package frc.team88.waypoints.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffyTunnelInterface;
import frc.team88.chassis.ChassisInterface;
import frc.team88.tunnel.TunnelServer;

public class WaitForCoprocessorPlan extends CommandBase {
    private final ChassisInterface chassis;
    private final DiffyTunnelInterface tunnel_interface;
    private boolean is_finished = false;
    
    public WaitForCoprocessorPlan(ChassisInterface chassis, DiffyTunnelInterface tunnel_interface)
    {
        this.chassis = chassis;
        this.tunnel_interface = tunnel_interface;
    }

    @Override
    public void initialize() {
        System.out.println("Starting WaitForCoprocessorPlan command");
    }

    @Override
    public void execute() {
        if (TunnelServer.anyClientsAlive() && tunnel_interface.isCommandActive()) {
            chassis.drive(tunnel_interface.getCommand());
        }
        
        switch (tunnel_interface.getGoalStatus()) {
            case RUNNING:
                break;
            case INVALID: 
                System.out.println("Coprocessor entered into state INVALID. Cancelling goal.");
                cancelGoal();
                break;
            case IDLE:
                System.out.println("Coprocessor entered into state IDLE. Cancelling goal.");
                cancelGoal();
                break;

            case FAILED:
                System.out.println("Coprocessor entered into state FAILED. Exiting command.");
                setFinished();
                break;
            case FINISHED:
                setFinished();
                break;
        }
    }

    private void cancelGoal()
    {
        is_finished = true;
        tunnel_interface.cancelGoal();
    }
    private void setFinished() {
        is_finished = true;
    }

    @Override
    public boolean isFinished() {
        return is_finished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("WaitForCoprocessorPlan finished.");
        if (interrupted) {
            System.out.println("WaitForCoprocessorPlan was interrupted. Cancelling goal.");
            cancelGoal();
        }
        chassis.holdDirection();
    }
}
