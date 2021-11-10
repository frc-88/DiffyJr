/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.DiffSwerveModule;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class ControllerUnitTest extends TimedRobot {
    DiffSwerveModule module = new DiffSwerveModule(new Translation2d(0.0, 0.0), 0, 1, 2, 3, 0.0);
    @Override
    public void robotInit() {
        System.out.println("Unit test initialized");
        module.setIdealState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(1.0)));
        for (int count = 0; count < 20; count++) {
            module.update();
            SwerveModuleState state = module.getState();
            System.out.println("State: " + state);
        }
    }

    @Override
    public void robotPeriodic() {
        System.exit(0);
    }
}
