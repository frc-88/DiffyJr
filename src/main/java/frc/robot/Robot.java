/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.subsystems.DiffyTunnelInterface;
import frc.robot.subsystems.SwerveNetworkTable;
import frc.team88.tunnel.TunnelServer;
import frc.robot.subsystems.swerve.Constants;
import frc.robot.subsystems.swerve.DiffSwerveChassis;
import frc.robot.subsystems.swerve.DiffSwerveModule;

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

        this.addPeriodic(this::controllerPeriodic, Constants.DifferentialSwerveModule.kDt, 0.0025);

        TunnelServer.instance.println("Diffy Jr is initialized");
    }

    @Override
    public void robotPeriodic() {
        // Happens after mode periodic method
        this.swerve.periodic();
        this.swerve_table.update();
    }

    public void disabledInit() {
        swerve.setEnabled(false);
    }

    @Override
    public void disabledPeriodic() {
        
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
        // this.swerve.holdDirection();
    }

    @Override
    public void teleopInit() {
        TunnelServer.instance.println("Diffy Jr teleop enabled");
        swerve.setEnabled(true);
    }

    public void controllerPeriodic() {
        this.swerve.controllerPeriodic();
        // this.swerve.backLeft.update();
    }

    public void testUpdate() {
        // TunnelServer.instance.writePacket("module",
        //     0,
        //     this.swerve.backLeft.getWheelVelocity(),
        //     this.swerve.backLeft.getModuleAngle(),
        //     this.swerve.backLeft.swerveControlLoop.getXHat(0),
        //     this.swerve.backLeft.swerveControlLoop.getXHat(1),
        //     this.swerve.backLeft.swerveControlLoop.getXHat(2),
        //     this.swerve.backLeft.swerveControlLoop.getObserver().getXhat(0),
        //     this.swerve.backLeft.swerveControlLoop.getObserver().getXhat(1),
        //     this.swerve.backLeft.swerveControlLoop.getObserver().getXhat(2),
        //     this.swerve.backLeft.swerveControlLoop.getError(0),
        //     this.swerve.backLeft.swerveControlLoop.getError(1),
        //     this.swerve.backLeft.swerveControlLoop.getError(2),
        //     this.swerve.backLeft.swerveControlLoop.getNextR(0),
        //     this.swerve.backLeft.swerveControlLoop.getNextR(1),
        //     this.swerve.backLeft.swerveControlLoop.getNextR(2),
        //     this.swerve.backLeft.reference.get(0, 0),
        //     this.swerve.backLeft.reference.get(1, 0),
        //     this.swerve.backLeft.reference.get(2, 0),
        //     this.swerve.backLeft.getLoNextVoltage(),
        //     this.swerve.backLeft.getHiNextVoltage()
        // );
        int time = (int)(RobotController.getFPGATime() / 1000000);
        double angle = 0.0;
        double speed = 0.0;
        switch (time % 4) {
            case 0: angle = 0.0; speed = 0.0; break;
            case 1: angle = 90.0; speed = 0.5; break;
            case 2: angle = 180.0; speed = 0.0; break;
            case 3: angle = 270.0; speed = -0.5; break;
        
            default:
                break;
        }
        for (DiffSwerveModule module : this.swerve.modules) {
            module.setModuleState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)));
        }
    }

    @Override
    public void teleopPeriodic() {
        // testUpdate();
        if (tunnel.anyClientsAlive() && diffy_interface.isCommandActive()) {
            swerve.drive(
                diffy_interface.getCommandVx(),
                diffy_interface.getCommandVy(),
                diffy_interface.getCommandVt(),
                false
            );
        }
        else if (this.gamepad.isConnected()) {
            // swerve.drive(
            //     MAX_SPEED * this.gamepad.getRawAxis(0),
            //     MAX_SPEED * this.gamepad.getRawAxis(1),
            //     MAX_ROTATION * this.gamepad.getRawAxis(4),
            //     false
            // );
        }
        else {
            // this.swerve.holdDirection();
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
