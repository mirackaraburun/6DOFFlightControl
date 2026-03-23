import numpy as np
from parameters import max_thrust, V_target, rho, S

K_v = 0.45
K_pitch = np.array([5.0, 2.0])

def control(state):
    u, v, w = state[0:3]
    q = state[4]
    theta = state[7]

    V = np.sqrt(u**2 + v**2 + w**2)

    # ---------------------------
    # TRIM THRUST (PHYSICS BASED)
    # ---------------------------
    alpha = np.arctan2(w, u)

    CD = 0.08 + 0.8 * alpha**2
    Drag = 0.5 * rho * V_target**2 * S * CD

    throttle_trim = Drag / max_thrust

    # ---------------------------
    # LQR VELOCITY CONTROL
    # ---------------------------
    #error_v = V_target - V
    #throttle = 0.4 + 0.05 * error_v
    #throttle = np.clip(throttle, 0.0, 1.0)
    # propeller-like model
    #thrust = max_thrust * throttle * (1 - V / 40.0)

    # negatif thrust olmasın
    #thrust = max(thrust, 0.0)
    thrust = 0
    # ---------------------------
    # PITCH LQR
    # ---------------------------
    x_pitch = np.array([theta, q])
    My_control = -K_pitch @ x_pitch

    return thrust, My_control