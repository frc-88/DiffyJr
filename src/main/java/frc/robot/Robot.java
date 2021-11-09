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
import frc.team88.swerve.SwerveController;
import frc.team88.swerve.module.SwerveModule;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private SwerveController swerve;
    private TunnelServer tunnel;
    private DiffyTunnelInterface diffy_interface;
    private SwerveNetworkTable swerve_table;

    private static final double MAX_SPEED = 14.7;
    private static final double MAX_ROTATION = 90.0;
    
    private Joystick gamepad;
    private static final int GAMEPAD_PORT = 0;
    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double CHANGE_DIRECTION_THRESHOLD = 0.25;
    private static final double HOLD_DIRECTION_TRANSLATION_THRESHOLD = 0.1;
    private static final double HOLD_DIRECTION_ROTATION_THRESHOLD = 1;  

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

        diffy_interface = new DiffyTunnelInterface(this.swerve);
        tunnel = new TunnelServer(diffy_interface, 5800, 15);
        
        tunnel.start();

        swerve_table = new SwerveNetworkTable(swerve);
        this.gamepad = new Joystick(GAMEPAD_PORT);

        TunnelServer.instance.println("Diffy Jr is initialized");
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
        this.swerve_table.update();
    }

    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (gamepad.getRawButton(4)) {
            this.swerve.setGyroYaw(0);
        }
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
        this.swerve.setBrake();
        TunnelServer.instance.println("Diffy Jr teleop enabled");
    }

    @Override
    public void teleopPeriodic() {
        if (tunnel.anyClientsAlive() && diffy_interface.isCommandActive()) {
            swerve.setVelocity(diffy_interface.getCommand());
        }
        else if (this.gamepad.isConnected()) {
            // Get the translation speed from the right trigger, scaled linearly so that fully pressed
            // commands our max speed in feet per second. Because the triggers on the XBox controller
            // actually go to zero when released, no deadband is needed.
            double translationSpeed = MAX_SPEED * this.gamepad.getRawAxis(3);

            // Get the rotation velocity from the right stick X axis, scaled linearly so that fully pushed
            // commands our max rotation speed in rotations per second. Uses a deadband since XBox
            // controller joysticks don't get exactly to zero when released.
            double rotationVelocity = this.gamepad.getRawAxis(4);
            if (Math.abs(rotationVelocity) < JOYSTICK_DEADBAND) {
                rotationVelocity = 0;
            }
            rotationVelocity *= MAX_ROTATION;

            // Set the translation speed and rotation velocities.
            this.swerve.setVelocity(translationSpeed, rotationVelocity);

            // Determine if the left stick is pressed enough to merit changing the direction.
            if (this.shouldChangeDirection()) {
                // The translation direction is field-centric if the right bumper is not pressed.
                boolean isFieldCentric = !this.gamepad.getRawButton(6);

                // Set the translation direction from the left stick.
                this.swerve.setTranslationDirection(this.calculateTranslationDirection(), isFieldCentric);

                // If we aren't changing translation direction, and we aren't commanding a significant speed
                // in
                // either translation or rotation, just hold the modules in their current position.
            }
            else if (translationSpeed < HOLD_DIRECTION_TRANSLATION_THRESHOLD
                && Math.abs(rotationVelocity) < HOLD_DIRECTION_ROTATION_THRESHOLD) {
                this.swerve.holdDirection();
            }
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


    /**
     * Calculates the angle of translation set by the left stick.
     *
     * @return The angle of translation, in degrees. 0 corresponds to forwards, and positive
     *     corresponds to counterclockwise.
     */
    private double calculateTranslationDirection() {
        // The x and y axis values. Y is inverted so that down is positive on XBox controllers, so we
        // need to invert it back.
        double x = gamepad.getRawAxis(0);
        double y = -gamepad.getRawAxis(1);

        // Calculate the angle. The variables are switched up because 0 degrees needs to be forwards.
        return Math.toDegrees(Math.atan2(x, -y));
    }

    /**
     * Determines if the left stick is pressed out far enough to merit changing the translation
     * direction. If the joystick is close to the center, it is too difficult to control the
     * direction.
     *
     * @return True if the current translation direction should be changed, false if it should stay
     *     the same.
     */
    private boolean shouldChangeDirection() {
        // The x and y axis values. Y is inverted so that down is positive on XBox controllers, so we
        // need to invert it back.
        double x = gamepad.getRawAxis(0);
        double y = -gamepad.getRawAxis(1);

        // Calculate the magnitude of the joystick position and use it as the threshold.
        return Math.sqrt(x * x + y * y) >= CHANGE_DIRECTION_THRESHOLD;
    }
}
