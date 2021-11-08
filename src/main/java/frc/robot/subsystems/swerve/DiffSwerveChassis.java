package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


public class DiffSwerveChassis {
    private final DiffSwerveModule frontRight;
    private final DiffSwerveModule frontLeft;
    private final DiffSwerveModule backRight;
    private final DiffSwerveModule backLeft;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private HolonomicDriveController controller;
    private ProfiledPIDController angleController;
    private double angleSetpoint = 0.0;

    private final NavX imu;

    public DiffSwerveChassis()
    {
        this.imu = new NavX();

        frontRight =
                new DiffSwerveModule(
                        Constants.DriveTrain.FRONT_RIGHT_POSITION,
                        RobotMap.CAN.TALONFX.FR_LO_FALCON,
                        RobotMap.CAN.TALONFX.FR_HI_FALCON,
                        RobotMap.CAN.CANIFIER,
                        RobotMap.DIO.ENCODER_FR,
                        Constants.DriveTrain.FRONT_RIGHT_ENCODER_OFFSET);
        frontLeft =
                new DiffSwerveModule(
                        Constants.DriveTrain.FRONT_LEFT_POSITION,
                        RobotMap.CAN.TALONFX.FL_LO_FALCON,
                        RobotMap.CAN.TALONFX.FL_HI_FALCON,
                        RobotMap.CAN.CANIFIER,
                        RobotMap.DIO.ENCODER_FL,
                        Constants.DriveTrain.FRONT_LEFT_ENCODER_OFFSET);
        backRight =
                new DiffSwerveModule(
                        Constants.DriveTrain.BACK_RIGHT_POSITION,
                        RobotMap.CAN.TALONFX.BR_LO_FALCON,
                        RobotMap.CAN.TALONFX.BR_HI_FALCON,
                        RobotMap.CAN.CANIFIER,
                        RobotMap.DIO.ENCODER_BR,
                        Constants.DriveTrain.BACK_RIGHT_ENCODER_OFFSET);
        backLeft =
                new DiffSwerveModule(
                        Constants.DriveTrain.BACK_LEFT_POSITION,
                        RobotMap.CAN.TALONFX.BL_HI_FALCON,
                        RobotMap.CAN.TALONFX.BL_LO_FALCON,
                        RobotMap.CAN.CANIFIER,
                        RobotMap.DIO.ENCODER_BL,
                        Constants.DriveTrain.BACK_LEFT_ENCODER_OFFSET);

        kinematics =
                new SwerveDriveKinematics(
                        frontLeft.getModuleLocation(),
                        frontRight.getModuleLocation(),
                        backLeft.getModuleLocation(),
                        backRight.getModuleLocation());
        odometry = new SwerveDriveOdometry(kinematics, getHeading());


        controller =
                new HolonomicDriveController(
                        new PIDController(
                                Constants.DriveTrain.kP,
                                Constants.DriveTrain.kI,
                                Constants.DriveTrain.kD),
                        new PIDController(
                                Constants.DriveTrain.kP,
                                Constants.DriveTrain.kI,
                                Constants.DriveTrain.kD),
                        new ProfiledPIDController(
                                Constants.DriveTrain.kP,
                                Constants.DriveTrain.kI,
                                Constants.DriveTrain.kD,
                                new TrapezoidProfile.Constraints(
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                        Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));
        angleController =
                new ProfiledPIDController(
                        Constants.DriveTrain.ANGLE_kP,
                        Constants.DriveTrain.ANGLE_kI,
                        Constants.DriveTrain.ANGLE_kD,
                        new TrapezoidProfile.Constraints(
                                Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL));
        angleController.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);
    }

    public void update() {
        odometry.update(
            getHeading(),
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }


    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdom(Pose2d pose) {
        odometry.resetPosition(pose, getHeading());
    }

    public double getVelocityX(){
        //Get the X veloctiy of the robot
        return imu.getVelocityX();
    }

    public double getVelocityY(){
        //Get the Y velocity of the robot
        return imu.getVelocityY();
    }

    public double getVelocityZ(){
        //Get the Z velocity of the robot
        return imu.getVelocityZ();
    }

    public ChassisSpeeds getChassisVelocity() {
        return kinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-imu.getYaw());
    }

    // Set wheel velocities to zero and hold module directions
    public void holdDirection() {
        frontRight.setIdealState(
            new SwerveModuleState(0, new Rotation2d(frontRight.getModuleAngle())));
        frontLeft.setIdealState(
                new SwerveModuleState(0, new Rotation2d(frontLeft.getModuleAngle())));
        backRight.setIdealState(
                new SwerveModuleState(0, new Rotation2d(backRight.getModuleAngle())));
        backLeft.setIdealState(
                new SwerveModuleState(0, new Rotation2d(backLeft.getModuleAngle())));
        resetAngleSetpoint();
    }

    private ChassisSpeeds getChassisSpeeds(double vx, double vy, double angularVelocity, boolean fieldRelative, Rotation2d relativeAngle) {
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, angularVelocity, relativeAngle);
        }
        else {
            chassisSpeeds = new ChassisSpeeds(vx, vy, angularVelocity);
        }
        return chassisSpeeds;
    }

    private void resetAngleSetpoint() {
        this.angleSetpoint = getHeading().getRadians();
        angleController.reset(this.angleSetpoint);
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx velocity in x direction
     * @param vy velocity in y direction
     * @param angularVelocity angular velocity (rotating speed)
     * @param fieldRelative forward is always forward no mater orientation of robot.
     */
    public void drive(double vx, double vy, double angularVelocity, boolean fieldRelative) {
        if (Math.abs(vx) < Constants.DriveTrain.DEADBAND
                && Math.abs(vy) < Constants.DriveTrain.DEADBAND
                && Math.abs(angularVelocity) < Constants.DriveTrain.DEADBAND) {
            // if setpoints are almost zero, set chassis to hold position
            holdDirection();
        }
        else if (Math.abs(angularVelocity) > 0) {
            // if translation and rotation are significant, push setpoints as-is
            ChassisSpeeds chassisSpeeds = getChassisSpeeds(vx, vy, angularVelocity, fieldRelative, getHeading());
            SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

            SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.DriveTrain.MAX_MPS);
            frontLeft.setIdealState(swerveModuleStates[0]);
            frontRight.setIdealState(swerveModuleStates[1]);
            backLeft.setIdealState(swerveModuleStates[2]);
            backRight.setIdealState(swerveModuleStates[3]);
            resetAngleSetpoint();
        }
        else {
            // if only translation is significant, set angular velocity according to previous angle setpoint
            double controllerAngVel = angleController.calculate(getHeading().getRadians(), angleSetpoint);
            ChassisSpeeds chassisSpeeds = getChassisSpeeds(vx, vy, controllerAngVel, fieldRelative, new Rotation2d(angleSetpoint));
            SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.DriveTrain.MAX_MPS);
            frontLeft.setIdealState(swerveModuleStates[0]);
            frontRight.setIdealState(swerveModuleStates[1]);
            backLeft.setIdealState(swerveModuleStates[2]);
            backRight.setIdealState(swerveModuleStates[3]);
        }
    }

    public void followPose(Pose2d pose, Rotation2d heading, double vel) {
        ChassisSpeeds adjustedSpeeds = controller.calculate(odometry.getPoseMeters(), pose, vel, heading);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.DriveTrain.MAX_MPS);
        frontLeft.setIdealState(moduleStates[0]);
        frontRight.setIdealState(moduleStates[1]);
        backLeft.setIdealState(moduleStates[2]);
        backRight.setIdealState(moduleStates[3]);
    }
}
