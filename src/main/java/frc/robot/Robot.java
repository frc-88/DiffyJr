/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.team88.swerve.SwerveController;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Joystick gamepad;

    private static final double MAX_SPEED = 14.7;
    private static final double MAX_ROTATION = 360;

    private static final int TRANS_X_AXIS_XBOX = 1;
    private static final int TRANS_Y_AXIS_XBOX = 0;
    // private static final int THROTTLE_AXIS_XBOX = 5;
    private static final int ROTATION_AXIS_XBOX = 4;
    private static final int DPAD_AXIS_XBOX = 0;
    private static final double XBOX_JOYSTICK_DEADZONE = 0.07;

    private double speedCapXbox = MAX_SPEED / 5.0;
    private double rotationCapXbox = MAX_ROTATION / 5.0;
    private int speedCapIndexXbox = 0;
    private int prevDpadValXbox = -1;

    private SwerveController m_swerve;

    @Override
    public void robotInit() {
        System.out.println("Diffy Jr is initializing!");
        m_swerve = new SwerveController("swerve.toml");
        System.out.println("Diffy Jr is ready!");
    }

    @Override
    public void robotPeriodic() {
        if (gamepad.getRawButton(7)) {
            // OOOOOOOH, I WONDER WHAT THIS DOES!
            // int infiniteTJSquareds = 88 / 0;
            // throw new IllegalStateException(
            // infiniteTJSquareds + " is too much awesome, I don't know how we didn't crash
            // already!");
        }
        m_swerve.update();
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
    }

    @Override
    public void teleopPeriodic() {
        double vx_joy_val = -gamepad.getRawAxis(TRANS_X_AXIS_XBOX);
        double vy_joy_val = -gamepad.getRawAxis(TRANS_Y_AXIS_XBOX);

        double vt_joy_val = -gamepad.getRawAxis(ROTATION_AXIS_XBOX);

        if (Math.abs(vx_joy_val) < XBOX_JOYSTICK_DEADZONE) {
            vx_joy_val = 0.0;
        }
        if (Math.abs(vy_joy_val) < XBOX_JOYSTICK_DEADZONE) {
            vy_joy_val = 0.0;
        }
        if (Math.abs(vt_joy_val) < XBOX_JOYSTICK_DEADZONE) {
            vt_joy_val = 0.0;
        }
        
        int dpad_val = gamepad.getPOV(DPAD_AXIS_XBOX);
        if (dpad_val != prevDpadValXbox) {
            if (dpad_val != -1) {
                updateXboxSpeedCaps(dpad_val);
            }
            prevDpadValXbox = dpad_val;
        }

        vx_joy_val *= speedCapXbox;
        vy_joy_val *= speedCapXbox;
        vt_joy_val *= rotationCapXbox;

        double velocity_dir = Math.atan2(vy_joy_val, vx_joy_val);
        double velocity_mag = Math.sqrt(vx_joy_val * vx_joy_val + vy_joy_val * vy_joy_val);

        // Set the target velocity
        if (vx_joy_val == 0.0 && vy_joy_val == 0.0 && vt_joy_val == 0.0) {
            m_swerve.holdDirection();
        }
        else {
            m_swerve.setVelocity(velocity_dir, velocity_mag, vt_joy_val, false);
        }
    }

    private void updateXboxSpeedCaps(int dpad_val)
    {
        switch (dpad_val) {
            case 180:
                speedCapIndexXbox -= 1;
                if (speedCapIndexXbox < 0) {
                    speedCapIndexXbox = 0;
                }
                break;
            case 0:
                speedCapIndexXbox += 1;
                if (speedCapIndexXbox > 4) {
                    speedCapIndexXbox = 4;
                }
                break;
            default:
                break;
        }

        switch (speedCapIndexXbox) {
            case 0: 
                speedCapXbox = MAX_SPEED / 5.0;
                rotationCapXbox = MAX_ROTATION / 5.0;
                break;
            case 1: 
                speedCapXbox = MAX_SPEED * 2.0 / 5.0;
                rotationCapXbox = MAX_ROTATION * 2.0 / 5.0;
                break;
            case 2: 
                speedCapXbox = MAX_SPEED * 3.0 / 5.0;
                rotationCapXbox = MAX_ROTATION * 3.0 / 5.0;
                break;
            case 3: 
                speedCapXbox = MAX_SPEED * 4.0 / 5.0;
                rotationCapXbox = MAX_ROTATION * 4.0 / 5.0;
                break;
            case 4: 
                speedCapXbox = MAX_SPEED;
                rotationCapXbox = MAX_ROTATION;
                break;
            default:
                break;
        }
        System.out.println("Setting speed cap to: " + speedCapXbox);
    }
}
