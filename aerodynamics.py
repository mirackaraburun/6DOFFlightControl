import numpy as np
from parameters import rho, S, c

def aero_coefficients(alpha):
    alpha_stall = np.deg2rad(15)

    if abs(alpha) < alpha_stall:
        CL = 5.5 * alpha
    else:
        CL = 5.5 * alpha_stall * np.sign(alpha)

    CD = 0.08 + 0.8 * alpha**2
    Cm = -0.8 * alpha

    return CL, CD, Cm


def aerodynamic_forces_moments(state):
    u, v, w = state[0:3]
    p, q, r = state[3:6]

    V = np.sqrt(u**2 + v**2 + w**2) + 1e-6
    alpha = np.arctan2(w, u)

    CL, CD, Cm = aero_coefficients(alpha)
    q_bar = 0.5 * rho * V**2

    Lift = q_bar * S * CL
    Drag = q_bar * S * CD
    Moment = q_bar * S * c * Cm

    # -----------------------------
    # WIND AXIS → BODY dönüşümü
    # -----------------------------
    # Wind frame:
    # Xw = -Drag
    # Zw = -Lift

    Xw = -Drag
    Zw = -Lift

    # Rotation (alpha only, beta=0)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    Fx = ca * Xw - sa * Zw
    Fz = sa * Xw + ca * Zw
    Fy = 0.0

    Mx = 0.0
    My = Moment
    Mz = 0.0

    return np.array([Fx, Fy, Fz]), np.array([Mx, My, Mz]), alpha