/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ros.SendCoprocessorGoals;
import frc.robot.commands.ros.WaitForCoprocessorPlan;
import frc.robot.commands.ros.WaitForCoprocessorRunning;
import frc.robot.subsystems.SwerveNetworkTable;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.tunnel.DiffyTunnelInterface;
import frc.robot.util.tunnel.TunnelServer;
import frc.team88.diffswerve.Constants;
import frc.team88.diffswerve.DiffSwerveChassis;
import frc.team88.diffswerve.Helpers;

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
    private CommandBase autoCommand;

    private static final int BUTTON_A = 5;
    private static final int BUTTON_B = 2;
    private static final int BUTTON_X = 3;
    private static final int BUTTON_Y = 4;

    private Joystick gamepad;
    private JoystickButton autoCommandButton = new JoystickButton(gamepad, BUTTON_B);
    private JoystickButton pursueButton = new JoystickButton(gamepad, BUTTON_A);
    private final int GAMEPAD_PORT = 0;
    private final double JOYSTICK_DEADBAND = 0.1;
    
    WaypointsPlan autoPlan;
    WaypointsPlan pursuePlan;

    @Override
    public void robotInit() {
        this.swerve = new DiffSwerveChassis();
        // this.swerve.setAngleControllerEnabled(false);

        diffy_interface = new DiffyTunnelInterface(this.swerve);
        tunnel = new TunnelServer(diffy_interface, 5800, 15);
        
        tunnel.start();

        autoPlan = new WaypointsPlan(diffy_interface);
        autoPlan.addWaypoint(new Waypoint("start").makeIgnoreWalls(true));
        autoPlan.addWaypoint(new Waypoint("point1").makeContinuous(true));
        autoPlan.addWaypoint(new Waypoint("point2"));
        autoPlan.addWaypoint(new Waypoint("power_cell"));
        autoPlan.addWaypoint(new Waypoint("power_cell"));

        pursuePlan = new WaypointsPlan(diffy_interface);
        pursuePlan.addWaypoint(new Waypoint("power_cell"));

        swerve_table = new SwerveNetworkTable(swerve);
        this.gamepad = new Joystick(GAMEPAD_PORT);

        this.addPeriodic(this::controllerPeriodic, Constants.DifferentialSwerveModule.kDt, 0.0025);

        autoCommandButton.whileActiveOnce(new SendCoprocessorGoals(autoPlan));
        autoCommandButton.whileActiveOnce(new SequentialCommandGroup(
            new SendCoprocessorGoals(pursuePlan),
            getWaitForCoprocessorPlan(20.0)
        ));
        
        TunnelServer.println("Diffy Jr is initialized");
    }

    public CommandBase getWaitForCoprocessorPlan(double waitTime) {
        CommandBase waitForPlanCommand = new SequentialCommandGroup(
            new ParallelRaceGroup(
                new WaitForCoprocessorRunning(diffy_interface),
                new WaitCommand(0.5)
            ),
            new WaitForCoprocessorPlan(swerve, diffy_interface)
        );
        if (waitTime <= 0.0) {
            return waitForPlanCommand;
        }
        else {
            return new ParallelRaceGroup(
                waitForPlanCommand,
                new WaitCommand(waitTime)
            );
        }
    }

    @Override
    public void robotPeriodic() {
        // Happens after mode periodic method
        this.swerve.periodic();
        this.swerve_table.update();
        CommandScheduler.getInstance().run();
    }

    public void disabledInit() {
        TunnelServer.println("Diffy Jr disabled");
        System.out.println("Diffy Jr disabled");
        swerve.setEnabled(false);
        diffy_interface.cancelGoal();
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void disabledPeriodic() {
        
    }

    @Override
    public void autonomousInit() {
        TunnelServer.println("Diffy Jr auto enabled");
        System.out.println("Diffy Jr auto enabled");
        swerve.setEnabled(true);
        autoCommand = getWaitForCoprocessorPlan(15.0);

        // schedule the autonomous command
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // this.swerve.holdDirection();
    }

    @Override
    public void teleopInit() {
        TunnelServer.println("Diffy Jr teleop enabled");
        System.out.println("Diffy Jr teleop enabled");
        swerve.setEnabled(true);
        diffy_interface.cancelGoal();
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    public void controllerPeriodic() {
        this.swerve.controllerPeriodic();
    }

    @Override
    public void teleopPeriodic() {
        if (TunnelServer.anyClientsAlive() && diffy_interface.isCommandActive()) {
            swerve.drive(diffy_interface.getCommand());
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
            swerve.drive(vx, vy, vt);
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
