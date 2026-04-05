# PID-Control Rocket Airbrake Flight Simulator

A physics-based flight simulator for optimizing airbrake deployment in model rockets to achieve a precise target apogee. Uses predictive PID control to dynamically adjust airbrakes during flight based on real-time apogee predictions.

---

## Overview

When a model rocket is in unpowered ascent, small airbrake fins can be extended to increase drag and bleed off altitude — allowing the rocket to hit a specific target apogee rather than overshooting it. This project simulates that process and automatically finds the optimal PID gains to minimize apogee error.

The controller does not react to current altitude. Instead, it continuously predicts where the rocket will end up given the current state and brake position, and adjusts the brakes to drive that prediction toward the target.

---

## How It Works

**Physics Model**

The simulator uses Euler integration with altitude-dependent air density (exponential atmosphere model) to propagate rocket state forward in time. At each timestep, drag force is computed as:

```
F_drag = 0.5 * ρ(h) * Cd * A(brake_position) * v²
```

where the effective drag area `A` scales linearly between a minimum (brakes closed) and maximum (brakes fully open) based on brake position.

**Apogee Prediction**

At every control step, a fast forward simulation runs from the current state to predict the final apogee under the current brake position. This predicted apogee — not current altitude — is the process variable fed into the PID controller.

**PID Control Loop**

```
error = predicted_apogee - target_apogee
adjustment = Kp*error + Ki*∫error dt + Kd*(Δerror/Δt)
new_brake_position = current_brake_position + adjustment  (clamped to [0, 1])
```

When the rocket is predicted to overshoot, the controller opens the brakes. When on track, it eases off.

**Gain Optimization**

Optimal PID gains are found automatically using `scipy.optimize.minimize` with the Nelder-Mead method, minimizing absolute apogee error over a full simulated flight.

---

## Repository Structure

```
├── Optimization.py               # Gain optimization and plotting
├── RocketAltitudeSimulator.py    # Physics simulation + apogee prediction
├── RocketPIDController.py        # Predictive PID controller
├── Simulator.py                  # Abstract base class for simulators
├── main.py                       # Entry point
└── README.md
```

---

## Usage

**1. Install dependencies**

```bash
pip install numpy scipy matplotlib
```

**2. Run with default rocket parameters**

```bash
python main.py
```

This will optimize PID gains for the configured rocket and target apogee, then simulate and plot the resulting flight.

**3. Configure your rocket**

Edit the `rocket_params` dict and `target_apogee` in `main.py`:

```python
rocket_params = {
    'h0': 76.25,       # Altitude at start of control (m) — typically motor burnout
    'v0': 70.25,       # Velocity at start of control (m/s)
    'mass': 0.595,     # Rocket mass (kg)
    'Cd': 0.65,        # Drag coefficient
    'A_base': 0.00452, # Drag area with brakes closed (m²)
    'A_max': 0.00524,  # Drag area with brakes fully open (m²)
}

target_apogee = 228.6  # meters
```

---

## Output

After optimization, three plots are generated:

- **Altitude** — actual trajectory vs. real-time apogee prediction vs. target
- **Velocity** — velocity over time through apogee
- **Brake position** — airbrake deployment profile over the flight

Optimized gains and final apogee error are printed to the console:

```
OPTIMIZATION RESULTS
Optimal PID Gains:
  Kp = 0.031724
  Ki = 0.004812
  Kd = 0.018563

Final error: 0.241m
Converged: True
Iterations: 87
```

---

## Architecture

| Class | File | Responsibility |
|---|---|---|
| `Simulator` | `Simulator.py` | Abstract base: `set_input`, `get_output`, `step` |
| `RocketAltitudeSimulator` | `RocketAltitudeSimulator.py` | Physics state + apogee prediction |
| `RocketPIDController` | `RocketPIDController.py` | Predictive PID control loop |
| `optimize_pid_gains` | `Optimization.py` | Nelder-Mead gain search |
| `simulate_and_plot` | `Optimization.py` | Full run + matplotlib visualization |

---

## Limitations & Future Work

- Atmosphere model is a simple exponential; a standard atmosphere table (ISA) could improve fidelity
- Gain optimization runs open-loop; a real deployment would require onboard state estimation (e.g. Kalman filter)
- No motor thrust modeling — simulation begins at burnout
- Servo dynamics and physical brake deployment lag are not modeled
