import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, step

# Tiempo de simulación
t = np.linspace(0, 10, 1000)

# --- Planta de segundo orden subamortiguada (ejemplo) ---
# G(s) = 1 / (s^2 + 2ζωn s + ωn^2), con ζ = 0.2 (subamortiguado), ωn = 2 rad/s
wn = 2
zeta = 0.2

# --- Controlador P ---
Kp = 1
num_p = [Kp * wn**2]
den_p = [1, 2*zeta*wn, wn**2]
system_p = lti(num_p, den_p)
_, y_p = step(system_p, T=t)

# --- Controlador PD ---
Kp = 2
Kd = 1
num_pd = [Kd, Kp * wn**2]
den_pd = [1, 2*zeta*wn + Kd, wn**2 + Kp]
system_pd = lti(num_pd, den_pd)
_, y_pd = step(system_pd, T=t)

# --- Controlador PID ---
Kp = 4
Kd = 1
Ki = 2
num_pid = [Kd, Kp, Ki]
den_pid = [1, 2*zeta*wn + Kd, wn**2 + Kp, Ki]
system_pid = lti(num_pid, den_pid)
_, y_pid = step(system_pid, T=t)

# --- Referencia (escalón unitario) ---
ref = np.ones_like(t)

# --- Graficar ---
plt.figure(figsize=(10, 5))
plt.plot(t, ref, 'k--', label='Referencia', linewidth=2)
plt.plot(t, y_p, 'r', label='Control P', linewidth=2)
plt.plot(t, y_pd, 'g', label='Control PD', linewidth=2)
plt.plot(t, y_pid, 'b', label='Control PID', linewidth=2)

plt.title('Comparación de Respuestas: Control P, PD y PID')
plt.xlabel('Tiempo (s)')
plt.ylabel('Salida del sistema')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
