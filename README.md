# Linear-Adaptive Model Predictive Controller (MPC)
<p align="justify ">
This repository is part of my diploma thesis titled <a href="http://dx.doi.org/10.26265/polynoe-7607">
Development of model predictive control schemes for unmanned aerial vehicles using neural networks </a>.
The current project focuses on MPC controller design for controlling the altitude dynamics of the quadrotor as well as the x-y plane position. As far as the orientation dynamics are concerned, custom discrete PID (DPID) controllers are used. The project is fully coded and simulated in MATLAB 2022b. Also, the optimization problem is solved using the SQP solver, and the
full-plant dynamics of the quad were solved using the ode15s solver. Finally, the benchmark trajectory for evaluating the controller's performance is a 3D helix.
text</p>

#### Notes:
* If you use this software, please cite it as below: </br>
<em> S. Papadopoulos, “Linear Adaptive MPC Scheme for Unmanned Aerial Vehicles Trajectory Tracking Control”, 2024, https://github.com/SpPap/Linear-Adaptive-MPC </em>

* The work related to this repository is presented in **Chapter 3** of the thesis document, where the optimization problems are analyzed in detail, the controller settings are described, and the complete simulation results are provided. The full text (in English) is available at: http://dx.doi.org/10.26265/polynoe-7607 </br>

* The controller is nonlinear, as it solves an optimization problem. The linear component arises from the linear state-space prediction models, while the adaptive nature stems from the fact that these models are time-varying.
  

## Control Scheme Diagram
<p align="center">
  <img src="https://github.com/SpPap/Linear-Adaptive-MPC/assets/52887728/64aae744-1b0a-493a-90f3-915e47a53148" width="75%">
</p>

## Tracking MPC Optimization Problem
Assume that the control input is denoted by $u_{k}$ and the state under control is $x$. Also, $f$ denotes the transition function, i.e., the prediction model. 
- $N_{p}$ is the Prediction Horizon 
- $N_{c}$ is the Control Horizon


Thus, we end up with the following optimal control problem (OCP) with $N_{c}$ design variables


$$
\begin{aligned}
\min_{\Delta u(k), ...,\Delta u(k+N_{c})} \quad & J = {\hat{\textbf{e}}}^T \textbf{Q}\ \hat{\textbf{e}} +  {\Delta 
 u_{k}}^T \textbf{R} \Delta u_{k} \\
   \textrm{s.t.} \quad & \ \  \textbf{x}^{+} = f(x_{k},u_{k}) & \\ 
  & \ \ \hat{\textbf{e}} = \textbf{x}^{r} - \hat{\textbf{x}}  \\
  &\ \ \Delta u_{min} \leq \Delta u_{k+i}  \leq \Delta u_{max} \ \ \ ,  \  i=0,...N_{c}  \\
  & \ \  u_{min} \leq   u_{k+i}  \leq  u_{max} \ \ \ \ \ \ \ \   , \  i=0,...N_{c}  \\
  &\ \ \Delta  u_{k+i} = 0  \ \ \ \ \ \ \ \ \ \ \ \ \ \  \ \ \ \ \ \  \  , \ i=N_{c}+1,...N_{p}    \\  
\end{aligned}
$$

## Simulation Results
- Simulation Environment => MATLAB 2022b

Reference signals for helix trajectory:

$$
\begin{cases}
x^r(t) = 0.5 \cos(0.5t) & \forall t \\
y^r(t) = 0.5 \sin(0.5t) & \forall t \\
z^r(t) = -1 - 0.1t     & \forall t \\
\psi^r(t) = 0         & \forall t
\end{cases}
$$

where $${x, y, z}$$ denote the linear positions, and $$\psi$$ represents the yaw angle.


| Animated | Static |
|-------------|-------------|
| ![](https://github.com/SpPap/Linear-Adaptive-MPC/assets/52887728/5316d342-7e54-4807-839e-28b7b844774a) | ![](https://github.com/SpPap/Linear-Adaptive-MPC/assets/52887728/044ed133-3c12-4610-af26-a2feb0806d20) |

