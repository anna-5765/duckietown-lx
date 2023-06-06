from typing import Tuple

import numpy as np

VERT_TOP = 0
VERT_FAR = 200
VERT_MID = 300
VERT_BOT = 480


def min_max_array(d):
    min = ((d<0)*d).sum(0)
    max = ((d>0)*d).sum(0)
    return (min, max)


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    res = np.zeros(shape=shape, dtype="float32")
    # these values populate left matrix
    
    # adding complexity
    # res[0:300, 0:120] = -1
    # res[0:300, 120:220] = -.5
    # res[0:300, 220:320] = -.3
    # # res[0:300, 320:440] = -1
    # # res[0:300, 440:540] = -.5
    # # res[0:300, 540:640] = -.3
    # res[200:360, 0:320] = .5
    # res[300:480, 0:320] = 1
    # res[300:480, 320:640] = -1

    # simple sol
    # res[0:480, 0:320] = 1
    # res[0:200, 0:320] = -1


    # physical duckiebot
    res[200:480, 0:320] = 1
    res[100:200, 0:320] = 0.5
    res[200:480, 320:640] = -0.5

    # ---
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    # res = np.zeros(shape=shape, dtype="float32")
    # # these values populate right matrix
    # res[0:300, 320:440] = -.3
    # res[0:300, 440:540] = -.5
    # res[0:300, 540:640] = -1
    # res[200:360, 320:640] = .5
    # res[360:480, 320:640] = 1
    # res[300:480, 0:320] = -1

    res = np.fliplr(get_motor_left_matrix(shape))
    
    # ---
    return res
