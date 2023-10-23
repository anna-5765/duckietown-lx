from typing import Tuple

import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """

    # Calculate the wheel rotation assuming no slip

    delta_ticks = ticks - prev_ticks
    dphi = delta_ticks * 2 * np.pi / resolution  # Wheel rotation per delta in radians


    return dphi, ticks


def pose_estimation(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    # Total distance traveled by each wheel
    d_left = R * delta_phi_left
    d_right = R * delta_phi_right

    # Find rotation and distance traveled by robot (robot frame)
    d_theta = (d_right - d_left) / baseline
    d_A = (d_right + d_left) / 2

    # Express movement in world frame
    d_x = d_A * np.cos(theta_prev)
    d_y = d_A * np.sin(theta_prev)

    # Add to previous position to find current

    x_curr = x_prev + d_x
    y_curr = y_prev + d_y
    theta_curr = theta_prev + d_theta

    return x_curr, y_curr, theta_curr

    # Duckietown Solutions
    
    # x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
    # y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
    # theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/(baseline)

    #  w = [R, 2*R / baseline, 1]

    # x = np.array(
    #     [
    #         [
    #             (delta_phi_left + delta_phi_right) * np.cos(theta_prev) / 2,
    #             (delta_phi_left + delta_phi_right) * np.sin(theta_prev) / 2,
    #             0,
    #         ],
    #         [0, 0, (delta_phi_right - delta_phi_left) / 2],
    #         [x_prev, y_prev, theta_prev],
    #     ]
    # )

