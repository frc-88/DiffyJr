#!/usr/bin/env python3

import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as fct
import control as ct

import math
import matplotlib.pyplot as plt
import numpy as np


class Constants:
    VOLTAGE = 12.0
    J_w = 0.005
    J_a = 0.004
    DIFF_MATRIX = np.array([
        [5.0/72.0, 7.0/72.0],
        [1.0/24.0, -1.0/24.0]
    ])
    Q_AZIMUTH_ANG_VELOCITY = 0.8  # radians per sec
    Q_WHEEL_ANG_VELOCITY = 5.0  # radians per sec
    Q_AZIMUTH = 1.5  # radians
    MODEL_AZIMUTH_ANGLE_NOISE = 0.1  # radians
    MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0  # radians per sec
    MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0  # radians per sec
    SENSOR_AZIMUTH_ANGLE_NOISE = 0.01  # radians
    SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1  # radians per sec
    SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1  # radians per sec


class DiffSwerve(fct.System):
    def __init__(self, dt):
        """Diff Swerve subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Azimuth", "rad"), ("Wheel velocity", "rad/s"), ("Azimuth velocity", ("rad/s"))]
        u_labels = [("Hi Voltage", "V"), ("Lo Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([
                [-Constants.VOLTAGE],
                [-Constants.VOLTAGE]]
            ),
            np.array([
                [Constants.VOLTAGE],
                [Constants.VOLTAGE]]
            ),
            dt,
            np.zeros((3, 1)),
            np.zeros((2, 1)),
        )

    def create_model(self, states, inputs):
        motor = fct.models.MOTOR_FALCON_500
        K_t = motor.Kt
        K_v = motor.Kv
        R = motor.R
        C_w = -K_t / (K_v * R * Constants.J_w)
        C_a = -K_t / (K_v * R * Constants.J_a)
        K_w = K_t / (R * Constants.J_w)
        K_a = K_t / (R * Constants.J_a)

        A = np.array([
            [0.0, 1.0, 0.0],
            [0.0, C_w, 0.0],
            [0.0, 0.0, C_a]
        ])

        B = np.array([
            [0.0, 0.0],
            [K_w * Constants.DIFF_MATRIX[0][0], K_w * Constants.DIFF_MATRIX[0][1]],
            [K_a * Constants.DIFF_MATRIX[1][0], K_a * Constants.DIFF_MATRIX[1][1]],
        ])
        C = np.ones((3, 3))
        D = np.zeros((3, 2))

        return ct.ss(A, B, C, D)

    def design_controller_observer(self):
        q = [
            Constants.Q_AZIMUTH,
            Constants.Q_WHEEL_ANG_VELOCITY,
            Constants.Q_AZIMUTH_ANG_VELOCITY
        ]
        r = [Constants.VOLTAGE, Constants.VOLTAGE]
        self.design_lqr(q, r)
        self.design_two_state_feedforward()

        q_model = [
            Constants.MODEL_AZIMUTH_ANGLE_NOISE,
            Constants.MODEL_WHEEL_ANG_VELOCITY_NOISE,
            Constants.MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
        ]
        r_model = [
            Constants.SENSOR_AZIMUTH_ANGLE_NOISE,
            Constants.SENSOR_WHEEL_ANG_VELOCITY_NOISE,
            Constants.SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
        ]
        self.design_kalman_filter(q_model, r_model)


def main():
    dt = 0.005
    diff_swerve = DiffSwerve(dt)
    # flywheel.export_cpp_coeffs("DiffSwerve", "subsystems/")
    # flywheel.export_java_coeffs("DiffSwerve")

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0], [0.0]])
        elif t[i] < l1:
            r = np.array([[np.pi], [0.0], [0.0]])
        else:
            r = np.array([[0.0], [0.0], [0.0]])
        refs.append(r)

    x_rec, ref_rec, u_rec, y_rec = diff_swerve.generate_time_responses(t, refs)
    diff_swerve.plot_time_responses(t, x_rec, ref_rec, u_rec)
    plt.gcf().set_size_inches(100, 50)

    if "--noninteractive" in sys.argv:
        plt.savefig("swerve_response.svg")
    else:
        plt.show()


if __name__ == "__main__":
    main()
