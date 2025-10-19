# PN_2D Simulator — Proportional Navigation Guidance (2D)

This repository implements a **2D Proportional Navigation (PN)** missile–target engagement simulator in MATLAB.

It models:
- Missile and target planar motion
- Classical PN guidance law
- Target maneuvers
- Actuator saturation
- Optional heading and seeker (LOS) errors
- Statistical sweeps over the navigation constant `N₀`

---

## ⚙️ 1. Time and Simulation Settings

| Parameter | Units | Description | Typical |
|------------|--------|--------------|----------|
| `dt` | s | Integration step size | 0.005–0.02 |
| `Tmax` | s | Maximum simulation time | 100–300 |

---

## 🚀 2. Guidance and Missile Dynamics

| Parameter | Units | Description | Typical |
|------------|--------|--------------|----------|
| `N0` | — | Navigation constant in PN law:<br>**aₙ = N₀·Vc·λ̇** | 3–6 |
| `a_max` | m/s² | Maximum lateral acceleration (saturation) | 50–200 |
| `use_actuator` | bool | Enables actuator saturation (`true` or `false`) | `true` |
| `tau_act` *(optional)* | s | 1st-order actuator lag constant (0 = none) | 0.05–0.2 |
| `tau_ldot` *(optional)* | s | Low-pass filter constant for measured λ̇ | 0.05–0.2 |

---

## 🛰️ 3. Initial Conditions

| Parameter | Units | Description | Example |
|------------|--------|-------------|----------|
| `m_pos0` | m | Missile initial position `[x; y]` | `[0; 2000]` |
| `m_vel0` | m/s | Missile initial velocity vector | `300*[cosd(5); sind(5)]` |
| `t_pos0` | m | Target initial position `[x; y]` | `[6000; 2000]` |
| `t_vel0` | m/s | Target initial velocity vector | `-250*[cosd(-10); sind(-10)]` |
| `m_heading_err_deg` | ° | Initial heading error on missile velocity | 0–2 |
| `t_heading_err_deg` | ° | Initial heading error on target velocity | 0–2 |

---

## 🎯 4. Target Maneuver Parameters

| Parameter | Units | Description | Example |
|------------|--------|-------------|----------|
| `t_maneuver_on` | s | Start time of target lateral maneuver | 5 |
| `t_maneuver_off` | s | End time of maneuver | 25 |
| `t_maneuver_acc` | m/s² | Lateral acceleration magnitude (⊥ to velocity) | 6 |

---

## 🧩 5. Termination and Thresholds

| Parameter | Units | Description | Typical |
|------------|--------|-------------|----------|
| `r_impact_threshold` | m | Impact radius (distance for intercept) | 3–10 |
| `min_Vc_for_tgo` | m/s | Minimum closing velocity for valid time-to-go | 1–10 |

### Definitions
- **Closing velocity:**  
  \[ V_c = -\frac{\mathbf{r}^\top \mathbf{v}_{rel}}{\|\mathbf{r}\|} \]  
  Positive when the missile is approaching.
- **Time-to-go:**  
  \[ t_{go} = \frac{R}{V_c}, \quad \text{if } V_c > \texttt{min\_Vc\_for\_tgo} \]

---

## 🎯 6. Seeker (LOS Sensor) Error Model

| Parameter | Units | Description | Typical |
|------------|--------|-------------|----------|
| `bias_lambda_deg` | ° | Constant LOS angle bias (boresight) | 0–1 |
| `sigma_lambda_deg` | ° | LOS angle white-noise standard deviation | 0–0.5 |
| `lambda_dot_tau` | s | Optional filter constant for λ̇ | 0.05–0.2 |

---

## 📋 7. Output Control

| Parameter | Type | Description |
|------------|-------|-------------|
| `print_summary` | bool | If `true`, prints a detailed encounter summary in the console. Always stored in `sim.summary`. |

---

## 📐 8. Internal Definitions

| Symbol | Definition |
|---------|-------------|
| **r** = t_pos − m_pos | Relative position vector |
| **v_rel** = t_vel − m_vel | Relative velocity |
| **λ** | Line-of-Sight (LOS) angle = atan2(r_y, r_x) |
| **λ̇** | LOS rate = (rₓvᵧ − r_yvₓ)/R² |
| **Vc** | Closing velocity = −(rᵀv_rel)/R |
| **uₗₒₛ**, **u_⊥** | Unit vectors along and perpendicular to LOS |
| **tgo** | Time-to-go = R / Vc (if Vc > min threshold) |
| **ZEM** | Zero Effort Miss ≈ (r + v_rel·tgo)ᵀu_⊥ |

---

## 📤 9. Output Structure (`sim`)

| Field | Units | Description |
|--------|--------|-------------|
| `t` | s | Time vector |
| `m_pos`, `t_pos` | m | Missile and target positions |
| `m_vel`, `t_vel` | m/s | Missile and target velocities |
| `range` | m | Separation distance |
| `Vc` | m/s | Closing velocity |
| `lambda`, `lambda_dot` | rad, rad/s | True LOS angle and rate |
| `lambda_meas`, `lambda_dot_meas` | rad, rad/s | Measured LOS and rate (with bias/noise/filter) |
| `a_cmd`, `a_act` | m/s² | Commanded and applied lateral accelerations |
| `ZEM` | m | Zero Effort Miss |
| `tgo` | s | Time-to-go (if valid) |
| `impact` | bool | `true` if within impact radius |
| `impact_time` | s | Impact time (NaN if no intercept) |
| `miss_distance` | m | Final distance at end or intercept |
| `summary` | string | Human-readable encounter summary |
| `params` | struct | Key parameters extracted from scenario |

---

## 📊 10. Example Usage

```matlab
s.dt = 0.01; s.Tmax = 240;
s.N0 = 3; s.a_max = 100; s.use_actuator = true;

s.m_pos0 = [0; 2000];
s.m_vel0 = 300 * [cosd(5); sind(5)];
s.t_pos0 = [6000; 2000];
s.t_vel0 = -250 * [cosd(-10); sind(-10)];

s.t_maneuver_on = 5;
s.t_maneuver_off = 25;
s.t_maneuver_acc = 6;

s.r_impact_threshold = 5;
s.min_Vc_for_tgo = 5;

% Optional errors
s.m_heading_err_deg = 1.0;
s.t_heading_err_deg = -0.5;
s.bias_lambda_deg = 0.5;
s.sigma_lambda_deg = 0.2;
s.lambda_dot_tau = 0.1;

s.print_summary = true;

sim = pn_2d_sim(s);
disp(sim.summary);
```

---

## 🧮 11. Typical Values

| Parameter | Recommended Range |
|------------|------------------|
| `dt` | 0.005–0.02 s |
| `N0` | 3–6 |
| `a_max` | 50–200 m/s² |
| `r_impact_threshold` | 3–10 m |
| `min_Vc_for_tgo` | 1–10 m/s |
| `bias_lambda_deg` | 0–1° |
| `sigma_lambda_deg` | 0–0.5° |
| `lambda_dot_tau` | 0.05–0.2 s |

---

## 📘 12. Notes

- Increasing **N₀** usually reduces miss distance but increases acceleration demand.  
- If the missile cannot achieve enough `a_max`, intercept may fail even at high N₀.  
- The **seeker noise and bias** allow testing of PN robustness under imperfect sensing.  
- The optional **time constant** `tau_ldot` helps smooth numerical differentiation of noisy LOS.

---

## 📄 13. Reference

This simulator follows the classical 2D proportional navigation formulation found in:
> Zarchan, P. *Tactical and Strategic Missile Guidance*, AIAA, 7th Ed., 2019.

---

**Author:** Miguel Ángel Gómez López  
