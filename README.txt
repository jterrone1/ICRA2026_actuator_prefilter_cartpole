
# The Actuator Pre-Filtering Approach to Control-Coherent Koopman LQR of Robot Systems Interacting with Compliant Environment

As a robot makes and breaks contact with environment surfaces, the equations of motion are switched. Task planning and real-time control become challenging as the system traverses multiple regions and switches the governing dynamics. This paper presents a modeling and real-time control methodology for such switched dynamical systems based on Koopman operator theory. Potentially, Koopman operators allow us to subsume segmented dynamics within a unified, globally linear model amenable for control analysis and synthesis. However, the original Koopman operators are not appliable to non-autonomous systems with exogenous input. A new method for converting robot dynamics to a Koopman-compatible model using actuator pre-filtering is presented and applied to the modeling and control of robots interacting with the environment. Specifically, an underactuated cart-pole robot bouncing against multiple walls is modeled as a Control-Coherent Koopman model and a Koopman LQR controller is designed for the wall-bouncing robot. The globally linear dynamic model reduces otherwise highly complex mixed-integer optimization to simple convex optimization. Simulation experiments demonstrate the effectiveness of the method and investigates the effect of the actuator pre-filter parameter on control performance.


## System Parameters

### Cart-Pole

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Pole Point Mass | m_p | 0.1 | kg |
| Cart Mass | m_c | 1 | kg |
| Pole Length | l | 0.5 | m |
| Gravity | g | 9.81 | m/s² |
| Simulation Time Step | Δt | 0.01 | s |

---

### Cart-Pole + Walls

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Wall Stiffness | k_w | 10 | N/m |
| Wall Location (relative to origin) | ±d | 0.2 | m |
| Torsional Stiffness | k | 2 | Nm/rad |
| Torsional Damping | b | 0.05 | Nms/rad |

---

### Cart-Pole Koopman

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Number of Training Data Points | N_p | 50000 | — |
| Number of Observables | M | 3 | — |
| RBF Shape Parameter | ε | 6 | — |

---

### Cart-Pole + Walls + Tors Koopman

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Number of Training Data Points | N_p | 50000 | — |
| Number of Observables | n | 9 | — |
| RBF Shape Parameter | ε | 1.1 | — |

---

### LQR Weights

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| x Error Weighting | Q_x, Q_ẋ | 100 | m⁻², s²m⁻² |
| θ Error Weighting | Q_θ, Q_θ̇ | 100 | rad⁻², s²rad⁻² |
| Force Weighting | Q_F | 0 | N⁻² |
| Observable Error Weighting | Q_g | 0 | — |
| Control Weighting | R | 1 | N⁻² |

## License

MIT License — see LICENSE file for details.