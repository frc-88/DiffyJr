package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.*;

public class DiffSwerveModule {
    private final TalonFX hiMotor;
    private final TalonFX loMotor;

    private final CANifiedPWMEncoder azimuthSensor;
    private final double azimuthOffset;  // angle in radians

    private final LinearSystemLoop<N3, N2, N3> swerveControlLoop;

    private Matrix<N3, N1> reference;
    private Matrix<N2, N1> input;

    private Matrix<N2, N2> diffMatrix;

    private boolean is_enabled = false;

    private Translation2d moduleLocation;

    public DiffSwerveModule(Translation2d moduleLocation, int loCanID, int hiCanID, int azimuthCanifierID, int azimuthPwmChannel, double azimuthOffsetRadians) {
        loMotor = initFalconMotor(loCanID);
        hiMotor = initFalconMotor(hiCanID);
        azimuthSensor = new CANifiedPWMEncoder(azimuthCanifierID, azimuthPwmChannel);
        azimuthOffset = azimuthOffsetRadians;

        diffMatrix = Matrix.mat(Nat.N2(), Nat.N2()).fill(
            Constants.DifferentialSwerveModule.GEAR_M11, Constants.DifferentialSwerveModule.GEAR_M12,
            Constants.DifferentialSwerveModule.GEAR_M21, Constants.DifferentialSwerveModule.GEAR_M22
        );

        swerveControlLoop = initControlLoop();
        input = VecBuilder.fill(0, 0);
        reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);

        this.moduleLocation = moduleLocation;
    }


    public void setEnabled(boolean enabled) {
        is_enabled = enabled;
    }
    
    private TalonFX initFalconMotor(int canID) {
        TalonFX motor = new TalonFX(canID);
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.setSensorPhase(false);
        motor.setNeutralMode(NeutralMode.Brake);

        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.DifferentialSwerveModule.TIMEOUT);
        motor.configForwardSoftLimitEnable(false);
        motor.configVoltageCompSaturation(Constants.DifferentialSwerveModule.VOLTAGE, Constants.DifferentialSwerveModule.TIMEOUT);
        motor.enableVoltageCompensation(true);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 5, Constants.DifferentialSwerveModule.TIMEOUT);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.DifferentialSwerveModule.TIMEOUT);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.DifferentialSwerveModule.TIMEOUT);
        motor.configForwardSoftLimitEnable(false);
        motor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                Constants.DifferentialSwerveModule.ENABLE_CURRENT_LIMIT,
                Constants.DifferentialSwerveModule.CURRENT_LIMIT,
                Constants.DifferentialSwerveModule.CURRENT_THRESHOLD,
                Constants.DifferentialSwerveModule.CURRENT_TRIGGER_TIME)
        );

        return motor;
    }

    private LinearSystemLoop<N3, N2, N3> initControlLoop() {

        // Creates a Linear System of our Differential Swerve Module.
        LinearSystem<N3, N2, N3> swerveModuleModel =
                createDifferentialSwerveModule(
                        DCMotor.getFalcon500(2),
                        Constants.DifferentialSwerveModule.INERTIA_STEER,
                        Constants.DifferentialSwerveModule.INERTIA_WHEEL,
                        diffMatrix);

        // Creates a Kalman Filter as our Observer for our module. Works since system is linear.
        KalmanFilter<N3, N2, N3> swerveObserver =
                new KalmanFilter<>(
                        Nat.N3(),
                        Nat.N3(),
                        swerveModuleModel,
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        Constants.DifferentialSwerveModule
                                                .MODEL_AZIMUTH_ANGLE_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .MODEL_WHEEL_ANG_VELOCITY_NOISE),
                        Matrix.mat(Nat.N3(), Nat.N1())
                                .fill(
                                        Constants.DifferentialSwerveModule
                                                .SENSOR_AZIMUTH_ANGLE_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
                                        Constants.DifferentialSwerveModule
                                                .SENSOR_WHEEL_ANG_VELOCITY_NOISE),
                        Constants.DifferentialSwerveModule.kDt);
        // Creates an LQR controller for our Swerve Module.
        LinearQuadraticRegulator<N3, N2, N3> swerveController =
                new LinearQuadraticRegulator<>(
                        swerveModuleModel,
                        // Q Vector/Matrix Maximum error tolerance
                        VecBuilder.fill(
                                Constants.DifferentialSwerveModule.Q_AZIMUTH,
                                Constants.DifferentialSwerveModule.Q_AZIMUTH_ANG_VELOCITY,
                                Constants.DifferentialSwerveModule.Q_WHEEL_ANG_VELOCITY),
                        // R Vector/Matrix Maximum control effort.
                        VecBuilder.fill(
                                Constants.DifferentialSwerveModule.CONTROL_EFFORT,
                                Constants.DifferentialSwerveModule.CONTROL_EFFORT),
                        Constants.DifferentialSwerveModule.kDt);

        // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max Volts,
        // Update Rate.
        LinearSystemLoop<N3, N2, N3> controlLoop =
                new LinearSystemLoop<>(
                        swerveModuleModel,
                        swerveController,
                        swerveObserver,
                        12.0,
                        Constants.DifferentialSwerveModule.kDt);

        // Initializes the vectors and matrices.
        controlLoop.reset(VecBuilder.fill(0, 0, 0));

        return controlLoop;
    }

    /**
     * Creates a StateSpace model of a differential swerve module.
     *
     * @param motor is the motor used.
     * @param Js is the Moment of Inertia of the steer component.
     * @param Jw is the Moment of Inertia of the wheel component.
     * @param Gs is the Gear Ratio of the steer.
     * @param Gw is the Gear Ratio of the wheel.
     * @return LinearSystem of state space model.
     */
    private static LinearSystem<N3, N2, N3> createDifferentialSwerveModule(DCMotor motor, double J_hi, double J_lo, Matrix<N2, N2> diffMatrix) {
        double K_t = motor.KtNMPerAmp;
        double K_v = motor.KvRadPerSecPerVolt;
        double R = motor.rOhms;
        double C_hi = -K_t / (K_v * R * J_hi);
        double C_lo = -K_t / (K_v * R * J_lo);
        double K_hi = K_t / (R * J_hi);
        double K_lo = K_t / (R * J_lo);

        Matrix<N2, N2> inverseDiffMatrix = diffMatrix.inv();
        Matrix<N2, N2> A_subset_constants = Matrix.mat(Nat.N2(), Nat.N2()).fill(
            C_hi, C_lo,
            C_hi, C_lo
        );
        Matrix<N2, N2> A_subset = A_subset_constants.times(inverseDiffMatrix);
        
        Matrix<N2, N2> B_subset_constants = Matrix.mat(Nat.N2(), Nat.N2()).fill(
            K_hi, K_lo,
            K_hi, K_lo
        );
        Matrix<N2, N2> B_subset = B_subset_constants.times(diffMatrix);

        var A = Matrix.mat(Nat.N3(), Nat.N3()).fill(
            0.0, 1.0, 0.0,
            0.0, A_subset.get(0, 0), A_subset.get(0, 1),
            0.0, A_subset.get(1, 0), A_subset.get(1, 1)
        );
        var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(
            0.0, 0.0,
            B_subset.get(0, 0), B_subset.get(0, 1),
            B_subset.get(1, 0), B_subset.get(1, 1)
        );
        var C = Matrix.mat(Nat.N3(), Nat.N3()).fill(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        );
        var D = Matrix.mat(Nat.N3(), Nat.N2()).fill(
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0
        );
        return new LinearSystem<>(A, B, C, D);
    }

    public void update() {
        // sets the next reference / setpoint.
        swerveControlLoop.setNextR(reference);
        // updates the kalman filter with new data points.
        swerveControlLoop.correct(
                VecBuilder.fill(
                        getModuleAngle(), getAzimuthAngularVelocity(), getWheelAngularVelocity()));
        // predict step of kalman filter.
        predict();
        if (is_enabled) {
            setFalconVoltage(hiMotor, getHiNextVoltage());
            setFalconVoltage(loMotor, getLoNextVoltage());
        }
    }


    // use custom predict() function for as absolute encoder azimuth angle and the angular velocity
    // of the module need to be continuous.
    private void predict() {
        // creates our input of voltage to our motors of u = K(r-x) but need to wrap angle to be
        // continuous
        // see wrapAngle().
        input =
                swerveControlLoop.clampInput(
                        swerveControlLoop
                                .getController()
                                .getK()
                                .times(
                                        wrapAngle(
                                                swerveControlLoop.getNextR(),
                                                swerveControlLoop.getXHat(),
                                                -Math.PI,
                                                Math.PI))
                                .plus(
                                        VecBuilder.fill(
                                                Constants.DifferentialSwerveModule.FEED_FORWARD
                                                        * reference.get(2, 0),
                                                -Constants.DifferentialSwerveModule.FEED_FORWARD
                                                        * reference.get(2, 0))));
        swerveControlLoop.getObserver().predict(input, Constants.DifferentialSwerveModule.kDt);
    }

    /**
     * wraps angle so that absolute encoder can be continues. (i.e) No issues when switching between
     * -PI and PI as they are the same point but different values.
     *
     * @param reference is the Matrix that contains the reference wanted such as [Math.PI, 0, 100].
     * @param xHat is the predicted states of our system. [Azimuth Angle, Azimuth Angular Velocity,
     *     Wheel Angular Velocity].
     * @param minAngle is the minimum angle in our case -PI.
     * @param maxAngle is the maximum angle in our case PI.
     */
    private Matrix<N3, N1> wrapAngle(
            Matrix<N3, N1> reference, Matrix<N3, N1> xHat, double minAngle, double maxAngle) {
        double angleError = reference.get(0, 0) - getModuleAngle();
        double positionError = MathUtil.inputModulus(angleError, minAngle, maxAngle);
        Matrix<N3, N1> error = reference.minus(xHat);
        return VecBuilder.fill(positionError, error.get(1, 0), error.get(2, 0));
    }


    public Translation2d getModuleLocation() {
        return moduleLocation;
    }


    private void setFalconVoltage(TalonFX motor, double voltage) {
        double limVoltage =
                Helpers.limit(
                        voltage,
                        -Constants.DifferentialSwerveModule.VOLTAGE,
                        Constants.DifferentialSwerveModule.VOLTAGE);
        motor.set(
                TalonFXControlMode.PercentOutput,
                limVoltage / Constants.DifferentialSwerveModule.VOLTAGE);
    }


    public double getModuleAngle() {
        return Helpers.boundHalfAngle(
                (azimuthSensor.getPosition() % (2.0 * Math.PI)) - azimuthOffset, true);
    }

    public double getWheelAngularVelocity() {
        // TODO
        return 0.0;
    }

    // Get module wheel velocity in meters per sec.
    public double getWheelVelocity() {
        return getWheelAngularVelocity()
                * Constants.DifferentialSwerveModule.WHEEL_RADIUS;
    }

    public double getPredictedWheelVelocity() {
        return getPredictedWheelAngularVelocity() * Constants.DifferentialSwerveModule.WHEEL_RADIUS;
    }

    public double getAzimuthAngularVelocity() {
        // TODO
        return 0.0;
    }

    public double getMotorRPM(TalonFX motor) {
        return motor.getSelectedSensorVelocity()
                / Constants.DifferentialSwerveModule.TICKS_TO_ROTATIONS
                * Constants.DifferentialSwerveModule.FALCON_RATE;
    }

    public double getMotorVoltage(TalonFX motor) {
        return motor.getMotorOutputVoltage();
    }

    
    public double getPredictedAzimuthAngularVelocity() {
        return swerveControlLoop.getObserver().getXhat(1);
    }

    public double getPredictedWheelAngularVelocity() {
        return swerveControlLoop.getXHat(2);
    }

    public double getPredictedAzimuthAngle() {
        return swerveControlLoop.getXHat(0);
    }

    public double getReferenceWheelAngularVelocity() {
        return swerveControlLoop.getNextR(2);
    }

    private void setReference(Matrix<N3, N1> reference) {
        this.reference = reference;
    }

    /**
     * gets the wanted voltage from our control law. u = K(r-x) our control law is slightly
     * different as we need to be continuous. Check method predict() for calculations.
     *
     * @return hi wanted voltage
     */
    public double getHiNextVoltage() {
        return input.get(0, 0);
    }

    public double getLoNextVoltage() {
        return input.get(1, 0);
    }

    public double getMotorCurrent(TalonFX motor) {
        return motor.getSupplyCurrent();
    }

    public double getReferenceModuleAngle() {
        return swerveControlLoop.getNextR(0);
    }

    public double getReferenceModuleAngularVelocity() {
        return swerveControlLoop.getNextR(1);
    }

    public double getReferenceWheelVelocity() {
        return swerveControlLoop.getNextR(2);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), new Rotation2d(getModuleAngle()));
    }


    /**
     * Sets the state of the module and sends the voltages wanted to the motors.
     *
     * @param state is the desired swerve module state.
     */
    public void setModuleState(SwerveModuleState state) {
        setReference(
                VecBuilder.fill(
                        state.angle.getRadians(),
                        swerveControlLoop.getXHat(1),
                        state.speedMetersPerSecond
                                / Constants.DifferentialSwerveModule.WHEEL_RADIUS));
    }


    /**
     * sets the modules to take the shorted path to the newest state.
     *
     * @param state azimuth angle in radians and velocity of wheel in meters per sec.
     */
    public void setIdealState(SwerveModuleState state) {
        // Rotation2d angleDifference = state.angle.minus(new Rotation2d(getModuleAngle()));
        // if (Math.abs(angleDifference.getRadians()) > (Math.PI / 2.0)) {
        //     setModuleState(
        //             new SwerveModuleState(
        //                     -state.speedMetersPerSecond,
        //                     state.angle.rotateBy(new Rotation2d(Math.PI))));
        // } else {
        //     setModuleState(state);
        // }
        setModuleState(SwerveModuleState.optimize(state, new Rotation2d(getModuleAngle())));
    }

}
