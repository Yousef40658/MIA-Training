import numpy as np
from pid import PidController   # keep your PID class file as is


# -----------------------------
# Simulation with safeguards
# -----------------------------
def simulate_pid(Kp, Ki, Kd, setpoint=10, steps=200, dt=0.01):
    pid = PidController(Kp, Ki, Kd, setpoint)
    position, velocity = 0, 0
    positions = []

    for _ in range(steps):
        control_signal = pid.compute(position, dt) 

        # Safety clamp to prevent runaway actuator output
        control_signal = np.clip(control_signal, -100, 100)

        velocity = control_signal
        position += velocity * dt                           #by check el error l kl point m3 values Kp , Ki , Kd mo5tlfa
        positions.append(position)

        # Early exit if diverging too far
        if abs(position) > 1e6:
            return 1e9, positions

    # Cost = mean squared error + oscillation penalty  
    errors = [(setpoint - p) ** 2 for p in positions]       
    mse = np.mean(errors)

    # Oscillation penalty (squared differences between consecutive positions)
    osc_penalty = np.mean(np.diff(positions) ** 2)

    return mse + 0.1 * osc_penalty, positions


# -----------------------------
# Auto tuning (grid search)
# -----------------------------
def auto_tune(setpoint=10):
    best_params = None
    best_cost = float("inf")

    # Coarse search grid
    for Kp in np.linspace(0.1, 20, 100):       # proportional
        for Ki in np.linspace(0.0, 6.0, 10):  # integral , 10 spaced numbers between 0 and 6
            for Kd in np.linspace(0.0, 1.0, 1000):  # derivative
                cost, _ = simulate_pid(Kp, Ki, Kd, setpoint)
                if cost < best_cost:
                    best_cost = cost
                    best_params = (Kp, Ki, Kd)

    return best_params, best_cost

#dh goz2 el tunning 3amtn , momkn yt7t f nfs el file 3ady -> len space in 1st line gives 100 spaced numbers between 0.1 and 20 

# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    (Kp, Ki, Kd), cost = auto_tune(setpoint=10)
    print(f"Best PID: Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}, with cost={cost:.6f}")
