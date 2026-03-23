import numpy as np

def wind_disturbance(t):
    # step + sinus gust
    wx = 2.0 * (t > 2)
    wy = 1.0 * np.sin(0.5 * t)
    wz = 0.5 * np.sin(1.0 * t)

    return np.array([wx, wy, wz])