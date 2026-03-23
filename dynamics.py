import numpy as np
from parameters import mass, g, I, I_inv
from aerodynamics import aerodynamic_forces_moments
from wind import wind_disturbance
from controller import control

def rotation_matrix(phi, theta, psi):
    cph, sph = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cps, sps = np.cos(psi), np.sin(psi)

    R = np.array([
        [cth*cps, sph*sth*cps - cph*sps, cph*sth*cps + sph*sps],
        [cth*sps, sph*sth*sps + cph*cps, cph*sth*sps - sph*cps],
        [-sth, sph*cth, cph*cth]
    ])
    return R


def euler_rates(p, q, r, phi, theta):
    tth = np.tan(theta)
    cth = np.cos(theta)

    return np.array([
        p + q*np.sin(phi)*tth + r*np.cos(phi)*tth,
        q*np.cos(phi) - r*np.sin(phi),
        q*np.sin(phi)/cth + r*np.cos(phi)/cth
    ])


def derivatives(t, state):
    u, v, w, p, q, r, phi, theta, psi = state

    vel = np.array([u, v, w])
    omega = np.array([p, q, r])

    # Wind
    wind = wind_disturbance(t)
    vel_rel = vel - wind

    # Aero
    F_aero, M_aero, alpha = aerodynamic_forces_moments(
        np.concatenate((vel_rel, omega))
    )

    # Control
    thrust, My_control = control(state)

    F_thrust = np.array([thrust, 0, 0])
    M_control = np.array([0, My_control, 0])
    print(f"Thrust: {thrust:.2f}, Drag approx: {-F_aero[0]:.2f}, u: {u:.2f}")
    # Gravity (body frame)
    R = rotation_matrix(phi, theta, psi)
    gravity_body = R.T @ np.array([0, 0, -mass * g])

    # Total forces
    F_total = F_aero + F_thrust + gravity_body

    # Translational dynamics
    vel_dot = (F_total / mass) - np.cross(omega, vel)

    # Rotational dynamics
    M_total = M_aero + M_control
    omega_dot = I_inv @ (M_total - np.cross(omega, I @ omega))

    # Euler angles
    euler_dot = euler_rates(p, q, r, phi, theta)
    # LATERAL DAMPING
    vel_dot[1] -= 2.5 * v
    omega_dot[2] -= 2.0 * r
    return np.concatenate((vel_dot, omega_dot, euler_dot)), alpha
    