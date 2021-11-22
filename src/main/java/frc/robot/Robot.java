/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DiffyTunnelInterface;
import frc.robot.subsystems.SwerveNetworkTable;
import frc.robot.subsystems.TrajectoryBuilder;
import frc.team88.tunnel.TunnelServer;
import frc.robot.subsystems.swerve.Constants;
import frc.robot.subsystems.swerve.DiffSwerveChassis;
import frc.robot.subsystems.swerve.Helpers;
import frc.robot.util.TestTrajectories;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private DiffSwerveChassis swerve;
    private TunnelServer tunnel;
    private DiffyTunnelInterface diffy_interface;
    private SwerveNetworkTable swerve_table;
    // private Trajectory auto_trajectory;
    // private CommandBase auto_command;
    private TrajectoryBuilder traj_builder;

    private Joystick gamepad;
    private static final int GAMEPAD_PORT = 0;
    private static final double JOYSTICK_DEADBAND = 0.1;
    
    @Override
    public void robotInit() {
        this.swerve = new DiffSwerveChassis();
        this.traj_builder = new TrajectoryBuilder();

        diffy_interface = new DiffyTunnelInterface(this.swerve, this.traj_builder);
        tunnel = new TunnelServer(diffy_interface, 5800, 15);
        
        tunnel.start();

        swerve_table = new SwerveNetworkTable(swerve);
        this.gamepad = new Joystick(GAMEPAD_PORT);

        this.addPeriodic(this::controllerPeriodic, Constants.DifferentialSwerveModule.kDt, 0.0025);

        // auto_trajectory = TestTrajectories.simpleTest1(this.swerve.getTrajectoryConfig());
        // auto_command = new FollowTrajectory(this.swerve, auto_trajectory, 1.0);

        TunnelServer.println("Diffy Jr is initialized");
    }

    @Override
    public void robotPeriodic() {
        // Happens after mode periodic method
        this.swerve.periodic();
        this.swerve_table.update();
        CommandScheduler.getInstance().run();
    }

    public void disabledInit() {
        System.out.println("Diffy Jr disabled");
        swerve.setEnabled(false);
        // auto_command.cancel();
    }

    @Override
    public void disabledPeriodic() {
        
    }

    @Override
    public void autonomousInit() {
        System.out.println("Diffy Jr autonomous enabled");
        // this.swerve.resetOdom();
        // auto_command.schedule();
        swerve.setEnabled(true);
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopInit() {
        TunnelServer.println("Diffy Jr teleop enabled");
        swerve.setEnabled(true);
        this.swerve.holdDirection();
        // auto_command.cancel();
    }

    public void controllerPeriodic() {
        this.swerve.controllerPeriodic();
    }

    @Override
    public void teleopPeriodic() {
        if (TunnelServer.anyClientsAlive() && diffy_interface.isCommandActive()) {
            swerve.drive(
                diffy_interface.getCommandVx(),
                diffy_interface.getCommandVy(),
                diffy_interface.getCommandVt(),
                false
            );
        }
        else if (this.gamepad.isConnected()) {
            double vx = Helpers.applyDeadband(-this.gamepad.getRawAxis(1), JOYSTICK_DEADBAND);
            double vy = Helpers.applyDeadband(-this.gamepad.getRawAxis(0), JOYSTICK_DEADBAND);
            double vt = Helpers.applyDeadband(-this.gamepad.getRawAxis(4), JOYSTICK_DEADBAND);

            vx *= Constants.DriveTrain.MAX_CHASSIS_SPEED;
            vy *= Constants.DriveTrain.MAX_CHASSIS_SPEED;
            vt *= Constants.DriveTrain.MAX_CHASSIS_ANG_VEL;

            // If magnitude of translation is in the "no-go" zone (_zero_epsilon..._min_linear_cmd),
            // set vx, vy to _min_linear_cmd with heading applied
            double trans_vel = Math.sqrt(vx * vx + vy * vy);
            if (Constants.EPSILON < Math.abs(trans_vel) && Math.abs(trans_vel) < Constants.DriveTrain.MIN_CHASSIS_SPEED)
            {
                double trans_angle = Math.atan2(vy, vx);
                vx = Constants.DriveTrain.MIN_CHASSIS_SPEED * Math.cos(trans_angle);
                vy = Constants.DriveTrain.MIN_CHASSIS_SPEED * Math.sin(trans_angle);
            }
            // If magnitude of translation is in the "zero" zone (<Constants.EPSILON),
            // Set translation velocity to zero
            //      If angular velocity is in the "no-go" zone,
            //      set vt to Constants.DriveTrain.MIN_CHASSIS_ANG_VEL with direction applied
            //      If angular velocity is in the "zero" zone,
            //      set vt to zero
            else if (Math.abs(trans_vel) < Constants.EPSILON) {
                vx = 0.0;
                vy = 0.0;
                if (Constants.EPSILON < Math.abs(vt) && Math.abs(vt) < Constants.DriveTrain.MIN_CHASSIS_ANG_VEL) {
                    vt = Math.signum(vt) * Constants.DriveTrain.MIN_CHASSIS_ANG_VEL;
                }
                else if (Math.abs(vt) < Constants.EPSILON) {
                    vt = 0.0;
                }
            }
            swerve.drive(vx, vy, vt, false);
        }
        else {
            this.swerve.holdDirection();
        }
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        this.swerve.holdDirection();
    }

    @Override
    public void simulationInit() {
    }
}
