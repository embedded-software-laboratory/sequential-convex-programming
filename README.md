# Sequential Convex Programming Methods for Real-time Optimal Trajectory Planning in Autonomous Vehicle Racing
<!-- icons from https://simpleicons.org/ -->
[![Paper](https://img.shields.io/badge/-Paper-00629B?logo=IEEE)](https://doi.org/10.1109/TIV.2022.3168130) 
[![Repository](https://img.shields.io/badge/-GitHub-181717?logo=GitHub)](https://github.com/embedded-software-laboratory/sequential-convex-programming) 
[![Video](https://img.shields.io/badge/-Video-FF0000?logo=YouTube)](https://youtu.be/7Iwh980JMCs) 
[![Open in Code Ocean](https://codeocean.com/codeocean-assets/badge/open-in-code-ocean.svg)](https://codeocean.com/capsule/6818033/tree)

![GIF showing SCR and SL](https://raw.githubusercontent.com/embedded-software-laboratory/sequential-convex-programming/master/animation_SCR_vs_SL.gif)
 
## Abstract
Optimization problems for trajectory planning in autonomous vehicle racing are characterized by their nonlinearity and nonconvexity. Instead of solving these optimization problems, usually a convex approximation is solved instead to achieve a high update rate. We present a real-time-capable model predictive control (MPC) trajectory planner based on a nonlinear single-track vehicle model and Pacejka's magic tire formula for autonomous vehicle racing. After formulating the general nonconvex trajectory optimization problem, we form a convex approximation using sequential convex programming (SCP). The state of the art convexifies track constraints using sequential linearization (SL), which is a method of relaxing the constraints. Solutions to the relaxed optimization problem are not guaranteed to be feasible in the nonconvex optimization problem. We propose sequential convex restriction (SCR) as a method to convexify track constraints. SCR guarantees that resulting solutions are feasible in the nonconvex optimization problem. We show recursive feasibility of solutions to the restricted optimization problem. The MPC is evaluated on a scaled version of the Hockenheimring racing track in simulation. The results show that an MPC using SCR yields faster lap times than an MPC using SL, while still being real-time capable.
 
## Instructions
0. clone or download this repo and open MATLAB in this folder
1. open folder 'code' via ```cd code``` in MATLAB
2. optional: choose a pre-configured scenario in file 'run.m'
    Alternatively, configure an individual configuration as described in
    the next paragraph
2. run the scenario via ```run()```

## Paper Results Reproduction
The script `code\+evaluation\paper.m` reproduces the simulation results. Afterwards, the results are available in the folder `results\`.
## Configuration
In the folder 'code/+config', all configurations of scenarios and vehicles are stored. You can combine the building blocks to your likes or even create a completely new configuration
 
## Requirements
- MATLAB 
    - Version: recommended R2021a, minimum R2019a
    - Symbolic Math Toolbox
    - Statistics and Machine Learning Toolbox (required for evaluation only)
- Solver
    - recommended IBM ILOG CPLEX Optimization Studio 12.10
    - alternatively, MATLAB's `quadprog` via 'Symbolic Math Toolbox'
 
tested on UNIX (Ubuntu 18.04 64-bit) and Windows 10 64-bit, MATLAB
R2021a, R2019b, R2019a

## Acknowledgements
This research is supported by the Deutsche Forschungsgemeinschaft (DFG, German Research Foundation) within the Priority Program SPP 1835 Cooperative Interacting Automobiles and the Post Graduate Program GRK 1856 Integrated Energy Supply Modules for Roadbound E-Mobility.

## Reference

<details>
<summary>
[1] P. Scheffe, T. Henneken, M. Kloock, B. Alrifaee.
"Sequential Convex Programming Methods for Real-time Optimal Trajectory Planning in Autonomous Vehicle Racing"
</summary>
<p>

```
@ARTICLE{scheffe2022sequential,
  author={Scheffe, Patrick and Henneken, Theodor Mario and Kloock, Maximilian and Alrifaee, Bassam},
  journal={IEEE Transactions on Intelligent Vehicles},
  title={Sequential Convex Programming Methods for Real-time Optimal Trajectory Planning in Autonomous Vehicle Racing},
  year={2022},
  volume={},
  number={},
  pages={1-1},
  doi={10.1109/TIV.2022.3168130}
}
```

</p>
</details>