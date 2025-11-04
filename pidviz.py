import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# start in fullscreen
fullscreen = True

# Simulationsparameter
t_end = 10.0
dt = 0.01
t = np.arange(0, t_end, dt)

# Prozessmodell (PT1-System)
def simulate_pid(setpoint, Kp, Ki, Kd, tau=1.0):
    y = np.zeros_like(t)
    u = np.zeros_like(t)
    e_prev = 0
    integral = 0

    for i in range(1, len(t)):
        e = setpoint[i] - y[i-1]
        integral += e * dt
        derivative = (e - e_prev) / dt
        e_prev = e

        u[i] = Kp * e + Ki * integral + Kd * derivative

        # PT1-System: dy/dt = (u - y)/tau
        y[i] = y[i-1] + dt * ((u[i] - y[i-1]) / tau)

    return y, u

# Anfangswerte PID
Kp0, Ki0, Kd0 = 1.0, 0.5, 0.1
t_switch0 = 1.0  # Startwert für den Rücksprung des Sollwerts

# Initiale Sollwerte
setpoint1 = np.ones_like(t)           # Sprung 0->1

setpoint2 = np.ones_like(t)
setpoint2[:] = 0
setpoint2[t >= 0.0] = 2               # Sprung auf 2 bei t=0
setpoint2[t >= t_switch0] = 1         # Rücksprung auf 1 nach t_switch0

# Systemantworten initial
y1, u1 = simulate_pid(setpoint1, Kp0, Ki0, Kd0)
y2, u2 = simulate_pid(setpoint2, Kp0, Ki0, Kd0)

# Plot einrichten
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.4)

line_y1, = ax.plot(t, y1, label="Sprung 0→1", lw=2)
line_y2, = ax.plot(t, y2, label="Sprung 0→2→1", lw=2, ls='--')
ax.plot(t, setpoint1, 'k:', label="Sollwert 0→1")
ax.set_xlabel("Zeit [s]")
ax.set_ylabel("Wert")
ax.set_ylim(0.0, 2.0)
ax.set_xlim(0.0, t_end)
ax.legend(loc="best")
ax.set_title("PID-Regler-Antwort auf Sollwertsprünge")

# Slider-Achsen
axcolor = 'lightgoldenrodyellow'
ax_Kp = plt.axes([0.1, 0.30, 0.8, 0.03], facecolor=axcolor)
ax_Ki = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor)
ax_Kd = plt.axes([0.1, 0.20, 0.8, 0.03], facecolor=axcolor)
ax_tswitch = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor=axcolor)

# Slider erstellen
s_Kp = Slider(ax_Kp, 'Kp', 0.0, 10.0, valinit=Kp0)
s_Ki = Slider(ax_Ki, 'Ki', 0.0, 2.0, valinit=Ki0)
s_Kd = Slider(ax_Kd, 'Kd', 0.0, 1.0, valinit=Kd0)
s_tswitch = Slider(ax_tswitch, 'Rücksprung t [s]', 0.0, t_end-1, valinit=t_switch0)

# Update-Funktion
def update(val):
    Kp = s_Kp.val
    Ki = s_Ki.val
    Kd = s_Kd.val
    t_switch = s_tswitch.val

    # Sollwert 0->1 (bleibt konstant)
    setpoint1 = np.ones_like(t)

    # Sollwert 0->2->1
    setpoint2 = np.ones_like(t)
    setpoint2[:] = 0
    setpoint2[t >= 0.0] = 2
    setpoint2[t >= t_switch] = 1

    # Simulation
    y1, _ = simulate_pid(setpoint1, Kp, Ki, Kd)
    y2, _ = simulate_pid(setpoint2, Kp, Ki, Kd)

    line_y1.set_ydata(y1)
    line_y2.set_ydata(y2)
    fig.canvas.draw_idle()

# Event-Handler
s_Kp.on_changed(update)
s_Ki.on_changed(update)
s_Kd.on_changed(update)
s_tswitch.on_changed(update)

# fullscreen
if fullscreen:
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

plt.show()
