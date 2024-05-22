# Linear-Adaptive-MPC

This repository is part of my diploma thesis titled "Development of Model Predictive Control (MPC) schemes for unmanned aerial vehicles using neural networks". 
The current project focuses on MPC controller design for controlling the altitude dynamics of the quadrotor as well as the x-y plane position. As far, as the orientation dynamics are concerned, custom discrete PID (DPID) controllers are used. The project is fully coded and simulated in MATLAB 2022b. Also, the optimization problem is solved using the SQP solver and the
full-plant dynamics of the quad were solved using Runge-Kutta (4,5) (ode45 solver) Finally, the benchmark trajectory for evaluating the controller's performance is a 3D helix.

## Control Scheme
![controlScheme_schematic](https://github.com/SpPap/Linear-Adaptive-MPC/assets/52887728/64aae744-1b0a-493a-90f3-915e47a53148)

## Tracking MPC Optimization Problem
Assume that the control input is denoted by $u_{k}$ and the state under control is $x$. Also, $f$ denotes the transition function, i.e. the prediction model. 
- $N_{p}$ is the Prediction Horizon 
- $N_{c}$ is the Control Horizon


Thus, we end up with the following QP problem with $N_{c}$ design variables


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

## Results

![3D_trajectory](https://github.com/SpPap/Linear-Adaptive-MPC/assets/52887728/5316d342-7e54-4807-839e-28b7b844774a)
![3D_trajectory](https://github.com/SpPap/Linear-Adaptive-MPC/assets/52887728/044ed133-3c12-4610-af26-a2feb0806d20)
