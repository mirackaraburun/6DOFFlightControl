import numpy as np

# Physical constants
g = 9.81
rho = 1.225  # air density

# UAV properties
mass = 4.643  # kg
S = 0.3465    # m^2
c = 0.225     # mean aerodynamic chord

# Inertia (convert from g*mm^2 to kg*m^2)
Ixx = 48270672.28 * 1e-9
Iyy = 760079183.98 * 1e-9
Izz = 947488423.10 * 1e-9
Ixy = -48270672.28 * 1e-9
Ixz = 34317021.89 * 1e-9
Iyz = -3837512.90 * 1e-9

I = np.array([
    [Ixx, -Ixy, -Ixz],
    [-Ixy, Iyy, -Iyz],
    [-Ixz, -Iyz, Izz]
])

I_inv = np.linalg.inv(I)

# Thrust
max_thrust = 48.0  # N

# Target speed
V_target = 22.0  # m/s