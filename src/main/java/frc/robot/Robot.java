/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

    private enum ControllerMode {
        kXBox,
        kXBox2,
        kFrsky;
    }
    private final ControllerMode controllerMode = ControllerMode.kXBox2;

    private SwerveController swerve;

    private Joystick gamepad;

    private static final double MAX_SPEED = 14.7;
    private static final double MAX_ROTATION = 90.;
    private static final double DEADZONE = 0.2;

    private final double[] SPEED_MODES = {
        0.65, 4.0, 7.0, 10.0, MAX_SPEED
    };
    private final double[] ROTATION_MODES = {
        10.0, 20.0, 30.0, 50.0, MAX_ROTATION
    };
    private int speedSelection = 0;
    private int prevDpadValue = 0;

    private BooleanSupplier zeroButton;
    private DoubleSupplier translationDirection;
    private BooleanSupplier maintainDirection;
    private DoubleSupplier translationSpeed;
    private DoubleSupplier rotationVelocity;
    private BooleanSupplier fieldCentricMode;
    private DoubleSupplier speedCap;
    private DoubleSupplier rotationCap;

    @Override
    public void robotInit() {
        this.swerve = new SwerveController("swerve.toml");
        this.swerve.setGyroYaw(0);

        this.gamepad = new Joystick(0);

        switch (this.controllerMode) {
            case kXBox:
                this.zeroButton = () -> gamepad.getRawButton(4);
                this.translationDirection = () -> {
                    double x = gamepad.getRawAxis(0);
                    double y = -gamepad.getRawAxis(1);
                    return Math.toDegrees(Math.atan2(y, x));
                };
                this.maintainDirection = () -> {
                    double x = gamepad.getRawAxis(0);
                    double y = -gamepad.getRawAxis(1);
                    return Math.sqrt(x*x + y*y) < 0.25;
                };
                this.translationSpeed = () -> gamepad.getRawAxis(3) * MAX_SPEED;
                this.rotationVelocity = () -> deadbandExponential(-gamepad.getRawAxis(4), 3, DEADZONE) * MAX_ROTATION;
                this.fieldCentricMode = () -> !gamepad.getRawButton(6);
                this.speedCap = () -> MAX_SPEED;
                this.rotationCap = () -> MAX_ROTATION;
                break;
            case kFrsky:
                this.zeroButton = () -> gamepad.getRawButton(4);
                this.translationDirection = () -> 0;
                this.maintainDirection = () -> true;
                this.translationSpeed = () -> 0;
                this.rotationVelocity = () -> 0;
                this.fieldCentricMode = () -> true;
                this.speedCap = () -> MAX_SPEED;
                this.rotationCap = () -> MAX_ROTATION;
                break;
            case kXBox2:
                this.zeroButton = () -> gamepad.getRawButton(4);
                this.translationDirection = () -> {
                    double x = gamepad.getRawAxis(0);
                    double y = -gamepad.getRawAxis(1);
                    return Math.toDegrees(Math.atan2(-x, y));
                };
                this.maintainDirection = () -> {
                    double x = gamepad.getRawAxis(0);
                    double y = -gamepad.getRawAxis(1);
                    return Math.sqrt(x*x + y*y) < 0.25;
                };
                this.translationSpeed = () -> {
                    double x = gamepad.getRawAxis(0);
                    double y = -gamepad.getRawAxis(1);
                    double mag = Math.sqrt(x*x + y*y);
                    if (Math.abs(mag) < DEADZONE) {
                        return 0.0;
                    }
                    else {
                        return Math.sqrt(x*x + y*y) * speedCap.getAsDouble();
                    }
                };
                this.rotationVelocity = () -> deadbandExponential(-gamepad.getRawAxis(4), 3, DEADZONE) * rotationCap.getAsDouble();
                this.fieldCentricMode = () -> gamepad.getRawButton(6);
                this.speedCap = () -> {
                    this.updateSpeedSelection(0);
                    return SPEED_MODES[speedSelection];
                };
                this.rotationCap = () -> {
                    this.updateSpeedSelection(0);
                    return ROTATION_MODES[speedSelection];
                };
                break;
        }
    }

    private void updateSpeedSelection(int pov) {
        int dpad = gamepad.getPOV(pov);
        if (prevDpadValue != dpad) {
            switch(dpad) {
                case 0: speedSelection++; break;
                case 180: speedSelection--; break;
            }
        }
        else {
            return;
        }
        if (speedSelection >= SPEED_MODES.length) {
            speedSelection = SPEED_MODES.length - 1;
        }
        else if (speedSelection < 0) {
            speedSelection = 0;
        }
        System.out.println(String.format("Speed selection: %d", speedSelection));
        prevDpadValue = dpad;
    }

    @Override
    public void robotPeriodic() {
        // Happens after mode periodic method
        this.swerve.update();
    }

    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (gamepad.getRawButton(7)) {
            this.swerve.setCoast();
        }
        if (gamepad.getRawButton(8)) {
            this.swerve.setBrake();
        }
        if (zeroButton.getAsBoolean()) {
            this.swerve.setGyroYaw(0);
        }
    }

    @Override
    public void autonomousInit() {
        this.swerve.setBrake();
    }

    @Override
    public void autonomousPeriodic() {
        this.swerve.holdDirection();
    }

    @Override
    public void teleopInit() {
        this.swerve.setBrake();
    }

    @Override
    public void teleopPeriodic() {
        this.swerve.activateMuxId(0);
        this.swerve.setVelocity(this.translationSpeed.getAsDouble(), this.rotationVelocity.getAsDouble());

        if (!this.maintainDirection.getAsBoolean()) {
            this.swerve.setTranslationDirection(this.translationDirection.getAsDouble(), this.fieldCentricMode.getAsBoolean());
        } else if (this.translationSpeed.getAsDouble() < 0.1 && Math.abs(this.rotationVelocity.getAsDouble()) < 0.1) {
            this.swerve.holdDirection();
        }
        this.swerve.controlFromNt();
    }

    @Override
    public void testInit() {
        this.swerve.setBrake();
    }

    @Override
    public void testPeriodic() {
        this.swerve.holdDirection();
    }

    @Override
    public void simulationInit() {
    }

    private static double signedPow(double base, int exp){
        double value = 0;
        
        if(base < 0 && exp%2==0){
            value = -Math.pow(base,exp);
        }
        else{
            value = Math.pow(base,exp);
        }

        return value;
    }
    
    private static double deadbandExponential(double spd, int exp, double deadband) {
        return Math.abs(spd)<deadband ? 0 : signedPow((spd - deadband*Math.signum(spd)) / (1 - deadband), exp);
    }
}
