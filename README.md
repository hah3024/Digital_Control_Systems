# Inverted Pendulum Control System

This project implements a control system for an inverted pendulum using MATLAB. It includes both continuous and discrete-time control strategies, such as state feedback and Linear Quadratic Regulator (LQR) control. The system is simulated for both linear and nonlinear dynamics, and the effects of sampling time on stability are analyzed.

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Description](#system-description)
3. [Control Strategies](#control-strategies)
4. [Code Structure](#code-structure)
5. [Running the Simulations](#running-the-simulations)
6. [Results](#results)
7. [Dependencies](#dependencies)
8. [License](#license)

---

## Project Overview
The goal of this project is to stabilize an inverted pendulum using various control techniques. The system is modeled in both continuous and discrete time, and simulations are performed to analyze the system's behavior under different control strategies.

---

## System Description
The inverted pendulum system consists of a cart with a pendulum attached to it. The system is described by the following parameters:
- **Mass (`M`)**: 1 kg
- **Pendulum Length (`L`)**: 0.842 m
- **Friction Coefficient (`F`)**: 1
- **Gravity (`g`)**: 9.8093 m/s²

The state vector `x` includes:
- `s`: Cart position
- `ds`: Cart velocity
- `phi`: Pendulum angle
- `dphi`: Pendulum angular velocity

---

## Control Strategies
The following control strategies are implemented:
1. **State Feedback Control**:
   - Designed using pole placement.
   - Simulated for both linear and nonlinear systems.
2. **LQR Optimal Control**:
   - Designed using the Linear Quadratic Regulator method.
   - Simulated for the nonlinear system.
3. **Discrete-Time Control**:
   - The system is discretized, and the effects of sampling time on stability are analyzed.

---

## Code Structure
The MATLAB code is organized as follows:
- **System Parameters**: Define the physical parameters of the system.
- **Linearized System**: Create the state-space representation of the linearized system.
- **State Feedback Controller**: Design and simulate the state feedback controller.
- **Nonlinear System Simulation**: Simulate the nonlinear dynamics using `ode45`.
- **Discrete-Time Control**: Analyze the effect of sampling time and implement discrete-time control.
- **LQR Control**: Design and simulate the LQR controller.
- **Reachability and Observability**: Check the system's reachability and observability.

---

## Running the Simulations
1. Open MATLAB and load the `pendulum.m` file.
2. Run the script to execute the simulations.
3. The results will be displayed in multiple plots, including:
   - System outputs (cart position and pendulum angle).
   - Control inputs.
   - Comparison of different control strategies.

---

## Results
The simulations produce the following results:
- **Closed-Loop System Response**: Plots of the cart position and pendulum angle over time.
- **Control Input**: The force applied to the cart to stabilize the pendulum.
- **Effect of Sampling Time**: Comparison of system stability for different sampling times.
- **LQR Control**: Optimal control performance for the nonlinear system.

---

## Dependencies
- MATLAB (tested on R2021a or later)
- Control System Toolbox

---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments
This project is based on classical control theory and is commonly used as a benchmark problem in control systems education.

---

For questions or feedback, please contact the project maintainer. 