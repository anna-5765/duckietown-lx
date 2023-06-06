from typing import Tuple

import numpy as np


def PIDController(
        v_0: float,
        theta_ref: float,
        theta_hat: float,
        prev_e: float,
        prev_int: float,
        delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing heading control.
    Args:
        v_0:        linear Duckiebot speed (given).
        theta_ref:  reference heading pose.
        theta_hat:  the current estiamted theta.
        prev_e:     tracking error at previous iteration.
        prev_int:   previous integral error term.
        delta_t:    time interval since last call.
    Returns:
        v_0:     linear velocity of the Duckiebot
        omega:   angular velocity of the Duckiebot
        e:       current tracking error (automatically becomes prev_e_y at next iteration).
        e_int:   current integral error (automatically becomes prev_int_y at next iteration).
    """

    # constant velocity (v_0), use controller to solve angular velocity omega
    # omega = u_t = kp * e + ki * e_int + kd * e_der

    # PID coefficient terms (sim)
    # # kp = 5
    # # ki = .2
    # # kd = .3
    # kp = 7
    # ki = .3
    # kd = .4

    # PID coefficient terms (duckiebot)
    kp = 15
    ki = 1
    kd = .2

    # Proportional term (tracking error = expected - actual)
    e = theta_ref - theta_hat

    # Integral term (finite sum of error to approximate integral)
    e_int = prev_int + e * delta_t

    # anti-windup from duckietown sol- preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)

    # Derivative term (finite difference in error over time "Euler backwards")
    e_der = (e - prev_e)/delta_t

    # PID equation
    omega = kp * e + ki * e_int + kd * e_der
    
    # Hint: print for debugging
    print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {omega} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    # ---
    return v_0, omega, e, e_int
