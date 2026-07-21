import numpy as np
import matplotlib.pyplot as plt

def integrate(v_prev, u, dt):
    """Integrate the input u over time to get the new value of v"""
    return v_prev + u * dt

def integrate_trapezoidal(v_prev, u, u_prev, dt):
    return v_prev + 0.5 * dt * (u + u_prev)

def integrate_trapezoidal_WRONG(v_prev, u, u_prev, dt):
    return 0.5 * dt * (u + u_prev)

v1_prev = 0.0
v2_prev = 0.0
u_prev = 0.0
dt = 0.01
signal = np.sin(np.linspace(0, 2 * np.pi, 100))
integrated_signal = []
integrated_signal_trap = []

for val in signal:
    integrated_signal.append(integrate(v1_prev, val, dt))
    integrated_signal_trap.append(integrate_trapezoidal(v2_prev, val, u_prev, dt))
    u_prev = val
    v1_prev = integrated_signal[-1]
    v2_prev = integrated_signal_trap[-1]
    
integrated_signal = np.array(integrated_signal)
integrated_signal_trap = np.array(integrated_signal_trap)

fig, (axt, axb) = plt.subplots(2, 1, sharex=True)
axt.plot(signal, label='Input Signal')
axt.set_ylabel('Input Signal')
axb.plot(integrated_signal, label='Integrated Signal (Simple)', color='orange')
axb.plot(integrated_signal_trap, label='Integrated Signal (Trapezoidal)', color='green')
axb.set_ylabel('Integrated Signal')
axb.set_xlabel('Time Steps')
axt.legend()
axb.legend()
plt.tight_layout()
plt.show()

