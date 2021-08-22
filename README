Sequential Convex Programming Methods for Real-time Trajectory Optimization in Autonomous Racing
---

![GIF showing SCR and SL](hhttps://raw.githubusercontent.com/embedded-software-laboratory/sequential-convex-programming/master/animation_SCR_vs_SL.gif)

[Paper (not published yet)]()

[![Open in Code Ocean](https://codeocean.com/codeocean-assets/badge/open-in-code-ocean.svg)](https://codeocean.com/capsule/2367434/tree)
 
[Watch Video on YouTube](https://youtu.be/21iETsolCNQ)
 
[Explore on GitHub](https://github.com/embedded-software-laboratory/sequential-convex-programming)
 
# Abstract
We present a real-time-capable Model Predictive Controller based on a
single-track vehicle model and Pacejka’s magic tire formula for autonomous
racing applications. After constructing a non-convex trajectory
optimization problem, it is convexified using the Sequential Convex
Programming method. We use two different methods for track discretization,
namely Sequential Linearization (SL) and Sequential Convex Restriction
(SCR). SL was already introduced in our previous paper. SCR is a new
addition, introduced in detail and its recursive feasibility proven.
We show that a controller with SCR yields faster lap times - for race
starts as well as flying laps - while being real-time capable.
 
# Instructions
1. open folder 'code' in MATLAB
2. run the file 'run.m'
    there you can choose a number from pre-configured scenarios.
    Alternatively, create a scenario from the existing configuration
    building blocks (scenarios and vehicles), or create a completly new
    configuration
 
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
print('-painters', gcf, "filename.svg", '-dsvg')
 
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