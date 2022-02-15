package frc.robot.util.tunnel;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.team88.diffswerve.DiffSwerveChassis;
import frc.team88.diffswerve.DiffSwerveModule;

public class DiffyTunnelInterface extends ROSInterface {
    private DiffSwerveChassis swerve;

    public DiffyTunnelInterface(DiffSwerveChassis swerve) {
        super((ChassisInterface)swerve);
        this.swerve = swerve;
    }

    @Override
    public void update() {
        super.update();
        // TunnelServer.writePacket("imu",
        //     this.swerve.imu.getYaw(), this.swerve.imu.getYawRate(),
        //     this.swerve.imu.getAccelX(), this.swerve.imu.getAccelY()
        // );
        DiffSwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            DiffSwerveModule module = modules[index];
            SwerveModuleState state = module.getState();
            TunnelServer.writePacket("joint",
                index,
                state.angle.getRadians()
            );
        }
    }
}
