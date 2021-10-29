/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.socket.ThreadedEchoServer;
import frc.robot.subsystems.SwerveNetworkTable;
import frc.robot.tunnel.TunnelDataRelayThread;
import frc.robot.tunnel.TunnelServer;
import frc.team88.swerve.SwerveController;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.state.VelocityState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private SwerveController swerve;
    // private SwerveNetworkTable m_swerve_table;
    private TunnelServer tunnel;
    private TunnelDataRelayThread data_relay_thread;
    // private ThreadedEchoServer echo_server;

    // private static final double MAX_SPEED = 14.7;
    // private static final double MAX_ROTATION = 360.;

    public enum SwitchingMode {
        kAlwaysSwitch,
        kNeverSwitch,
        kSmart,
    }

    private final SwitchingMode SWITCHING_MODE = SwitchingMode.kAlwaysSwitch;

    @Override
    public void robotInit() {
        this.swerve = new SwerveController("swerve.toml");
        this.swerve.setGyroYaw(0);
        this.swerve.setAzimuthWrapBiasStrategy((SwerveModule module) -> diffyJrAzimuthWrapStrategy(module));

        // m_swerve_table = new SwerveNetworkTable(swerve);
        // echo_server = new ThreadedEchoServer();
        // echo_server.start();
        tunnel = new TunnelServer(swerve, 3000);
        tunnel.start();

        data_relay_thread = new TunnelDataRelayThread(tunnel);
        data_relay_thread.start();
    }

    public double diffyJrAzimuthWrapStrategy(SwerveModule module) {
        switch (SWITCHING_MODE) {
            case kAlwaysSwitch:
                return 90;
            case kNeverSwitch:
                return 180;
            case kSmart:
                double currentVelocity = module.getWheelVelocity();
                if (module.getAzimuthVelocity() > 30) {
                    return 180;
                } else if (currentVelocity < 2.5) {
                    return 90;
                } else if (currentVelocity < 5.5) {
                    return 120;
                } else {
                    return 180;
                }
        }
        throw new IllegalStateException("Switching mode is not supported");
    }

    @Override
    public void robotPeriodic() {
        // Happens after mode periodic method
        this.swerve.update();
        // m_swerve_table.update();
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
    }

    @Override
    public void teleopInit() {
        this.swerve.setBrake();
    }

    @Override
    public void teleopPeriodic() {
        if (!tunnel.setCommandIfActive()) {
            swerve.holdDirection();
        }
        // if (m_swerve_table.isCommandActive()) {
        //     m_swerve_table.setCommand();
        // }
        // 
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }
}
