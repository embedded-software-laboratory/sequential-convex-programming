Sequential Convex Programming Methods for Real-time Optimal Trajectory Planning in Autonomous Vehicle Racing
---

![GIF showing SCR and SL](https://raw.githubusercontent.com/embedded-software-laboratory/sequential-convex-programming/master/animation_SCR_vs_SL.gif)

[Paper (not published yet)]()

[![Open in Code Ocean](https://codeocean.com/codeocean-assets/badge/open-in-code-ocean.svg)](https://codeocean.com/capsule/6818033/tree/v1)

[Watch Full Video on YouTube](https://youtu.be/21iETsolCNQ)
 
[Explore on GitHub](https://github.com/embedded-software-laboratory/sequential-convex-programming)
 
# Abstract
We present a real-time-capable Model Predictive Controller (MPC)
based on a non-linear single-track vehicle model and Pacejka’s
magic tire formula for autonomous racing applications. After 
formulating the general non-convex trajectory optimization 
problem, the model is linearized around estimated operating 
points and the constraints are convexified using the Sequential
Convex Programming (SCP) method. We use two different methods
to convexify the non-convex track constraints, namely Sequential
Linearization (SL) and Sequential Convex Restriction (SCR).
SL, a method of relaxing the constraints, was introduced in our
previous paper. SCR, a method of restricting the constraints,
is introduced in this paper. We show the application of SCR to
autonomous racing and prove that it does not interfere with
recursive feasibility. We compare the predicted trajectory
quality of the non-linear single-track model with the linear
double integrator model from our previous paper. The MPC
performance is evaluated on a scaled version of the Hockenheimring
racing track. We show that an MPC with SCR yields faster lap times
than an MPC with SL - for race starts as well as flying laps -
while still being real-time capable.
 
# Instructions
0. clone or download this repo and open MATLAB in this folder
1. open folder 'code' via ```cd code``` in MATLAB
2. optional: choose a pre-configured scenario in file 'run.m'
    Alternatively, configure an individual configuration as described in
    the next paragraph
2. run the scenario via ```run()```

## Configuration
In the folder 'code/+config', all configruations of scenarios and vehicles
are stored. You can combine the building blocks to your likes or even
create a completly new configuration
 
## Requirements
- MATLAB 
    - Version: recommended R2021a, minimum R2019a
    - Symbolic Math Toolbox
    - Statistics and Machine Learning Toolbox (required for evaluation only)
- Solver
    - recommended IBM ILOG CPLEX Optimization Studio 12.10
    - alternatively, MATLAB's 'quadprog' via 'Symbolic Math Toolbox'
 
tested on UNIX (Ubuntu 18.04 64-bit) and Windows 10 64-bit, MATLAB
R2021a, R2019b, R2019a
 
## Miscellaneous
- export large figures:
```print('-painters', gcf, "filename.svg", '-dsvg')```
 
## TODOs
- also search for 'FIXME' for issues to be fixed
- use 'sim scale' to norm all other scales
    - regularize states and inputs and their respective weights, scale models and track --> allows easy adaption to miniature and full-sized vehicle
    - scale vehicle size (length/width) for correct vehicle avoidance
- for usage of differing models for simulation/controller: improve acceleration controller (linear model inputs --> simulation inputs)
- geometrical vehicle scaling is missing: values hard-coded in 'vehicle' config, better should be part of vehicle model parameters
- smoke tests by running all scenarios automatically
- track models should give available start positions & max number of vehicles
- add acceleration limits to plot for linear model