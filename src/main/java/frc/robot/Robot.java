/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DiffyTunnelInterface;
import frc.robot.subsystems.SwerveNetworkTable;
import frc.team88.tunnel.TunnelServer;
import frc.robot.subsystems.swerve.DiffSwerveChassis;

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

    private static final double MAX_SPEED = 14.7;
    private static final double MAX_ROTATION = 90.0;
    
    private Joystick gamepad;
    private static final int GAMEPAD_PORT = 0;
    
    @Override
    public void robotInit() {
        this.swerve = new DiffSwerveChassis();

        diffy_interface = new DiffyTunnelInterface(this.swerve);
        tunnel = new TunnelServer(diffy_interface, 5800, 15);
        tunnel.start();

        swerve_table = new SwerveNetworkTable(swerve);
        this.gamepad = new Joystick(GAMEPAD_PORT);
    }

    @Override
    public void robotPeriodic() {
        // Happens after mode periodic method
        this.swerve.update();
        this.swerve_table.update();
    }

    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
        this.swerve.holdDirection();
    }

    @Override
    public void teleopInit() {
        
    }

    @Override
    public void teleopPeriodic() {
        if (tunnel.anyClientsAlive() && diffy_interface.isCommandActive()) {
            swerve.drive(
                diffy_interface.getCommandVx(),
                diffy_interface.getCommandVy(),
                diffy_interface.getCommandVt(),
                false
            );
        }
        else if (this.gamepad.isConnected()) {
            swerve.drive(
                MAX_SPEED * this.gamepad.getRawAxis(0),
                MAX_SPEED * this.gamepad.getRawAxis(1),
                MAX_ROTATION * this.gamepad.getRawAxis(4),
                false
            );
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
