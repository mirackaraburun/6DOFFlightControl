import numpy as np
import matplotlib.pyplot as plt
from dynamics import derivatives

dt = 0.01
T = 20
N = int(T / dt)

# state: [u v w p q r phi theta psi]
state = np.array([20, 0, 0, 0, 0, 0, 0, 0, 0])

history = []
alpha_hist = []
time = []
def rk4_step(f, t, y, dt):
    k1, _ = f(t, y)
    k2, _ = f(t + dt/2, y + dt/2 * k1)
    k3, _ = f(t + dt/2, y + dt/2 * k2)
    k4, _ = f(t + dt, y + dt * k3)

    return y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
for i in range(N):
    t = i * dt

    state = rk4_step(derivatives, t, state, dt)
    _, alpha = derivatives(t, state)

    history.append(state.copy())
    alpha_hist.append(alpha)
    time.append(t)

history = np.array(history)

u = history[:, 0]
v = history[:, 1]
w = history[:, 2]
p = history[:, 3]
q = history[:, 4]
r = history[:, 5]
theta = history[:, 7]

V = np.sqrt(u**2 + v**2 + w**2)

# --- PLOTS ---

plt.figure()
plt.plot(time, V)
plt.title("Velocity vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.grid()

plt.figure()
plt.plot(time, q)
plt.title("Pitch Rate (q) vs Time")
plt.xlabel("Time (s)")
plt.ylabel("rad/s")
plt.grid()

plt.figure()
plt.plot(time, theta)
plt.title("Pitch Angle vs Time")
plt.xlabel("Time (s)")
plt.ylabel("rad")
plt.grid()

plt.figure()
plt.plot(time, alpha_hist)
plt.title("Angle of Attack vs Time")
plt.xlabel("Time (s)")
plt.ylabel("rad")
plt.grid()

plt.figure()
plt.plot(time, p, label="p")
plt.plot(time, q, label="q")
plt.plot(time, r, label="r")
plt.legend()
plt.title("Angular Rates vs Time")
plt.grid()

plt.show()
print("Final u:", u[-1])
print("Final v:", v[-1])
print("Final w:", w[-1])