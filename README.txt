
# The Actuator Pre-Filtering Approach to Control-Coherent Koopman LQR of Robot Systems Interacting with Compliant Environment

As a robot makes and breaks contact with environment surfaces, the equations of motion are switched. Task planning and real-time control become challenging as the system traverses multiple regions and switches the governing dynamics. This paper presents a modeling and real-time control methodology for such switched dynamical systems based on Koopman operator theory. Potentially, Koopman operators allow us to subsume segmented dynamics within a unified, globally linear model amenable for control analysis and synthesis. However, the original Koopman operators are not appliable to non-autonomous systems with exogenous input. A new method for converting robot dynamics to a Koopman-compatible model using actuator pre-filtering is presented and applied to the modeling and control of robots interacting with the environment. Specifically, an underactuated cart-pole robot bouncing against multiple walls is modeled as a Control-Coherent Koopman model and a Koopman LQR controller is designed for the wall-bouncing robot. The globally linear dynamic model reduces otherwise highly complex mixed-integer optimization to simple convex optimization. Simulation experiments demonstrate the effectiveness of the method and investigates the effect of the actuator pre-filter parameter on control performance.



## License

MIT License — see LICENSE file for details.