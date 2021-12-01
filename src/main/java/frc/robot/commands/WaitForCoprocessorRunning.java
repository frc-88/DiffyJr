package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffyTunnelInterface;
import frc.robot.util.GoalStatus;

public class WaitForCoprocessorRunning extends CommandBase {
    private final DiffyTunnelInterface tunnel_interface;
    
    public WaitForCoprocessorRunning(DiffyTunnelInterface tunnel_interface) {
        this.tunnel_interface = tunnel_interface;
    }


    @Override
    public void initialize() {
        tunnel_interface.executeGoal();
        System.out.println("Starting WaitForCoprocessorRunning command");
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return tunnel_interface.getGoalStatus() != GoalStatus.RUNNING;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("WaitForCoprocessorRunning finished");
        if (!tunnel_interface.isGoalStatusActive()) {
            System.out.println("Goal status message is stale! Canceling WaitForCoprocessorRunning");
            tunnel_interface.cancelGoal();
        }
    }
}
