import numpy as np
from math import sqrt

def Jacobian(x):
    px, py, vx, vy = x
    if px == 0 and py == 0:
        print("Error: both px and py are zero while trying to")
        print("       calculate the Jacobian.")
        return np.zeros((3, 4))
    # Prepare calculation
    c1 = px * px + py * py
    c2 = sqrt(c1)
    c3 = c1 * c2
    # Fill in the matrix
    Hj = np.array([
        [px / c2, py / c2, 0.0, 0.0],
        [-py / c1, px / c1, 0.0, 0.0],
        [py * (vx * py - vy * px) / c3,
         px * (vy * px - vx * py) / c3,
         px / c2,
         py / c2]
    ])
    return Hj

# code from https://www.programcreek.com/python/?CodeExample=normalize+angle
def normalizeAngle(angle):
    """
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle 
