% main file to run: 'run.m' in folder 'code'
%   there you can choose scenarios or indicually build one
%
%   make sure 'code' is your matlab working folder

%% Requirements
% MATLAB 
%   - Version: recommended R2021a, minimum R2019a
%   - Symbolic Math Toolbox
%   - Statistics and Machine Learning Toolbox (required for evaluation only)
%   - solver:
%       - recommended IBM ILOG CPLEX Optimization Studio 12.10
%       - alternatively MATLAB's 'quadprog' via 'Symbolic Math Toolbox'
%
% tested on UNIX (Ubuntu 18.04 64-bit) and Windows 10 64-bit, MATLAB
% R2021a, R2019b, R2019a

%% TODOs
% - also search for 'FIXME' for issues to be fixed
% - use 'sim scale' to norm all other scales
%     - regularize states and inputs and their respective weights, scale models and track --> allows easy adaption to miniature and full-sized vehicle
%     - scale vehicle size (length/width) for correct vehicle avoidance
% - for usage of differing models for simulation/controller: improve acceleration controller (linear model inputs --> simulation inputs)
% - SCR Paper: check scripts con2vert etc.
% - geometrical vehicle scaling is missing: values hard-coded in 'vehicle' config, better should be part of vehicle model parameters
% - smoke tests by running all scenarios automatically
% - track models should give available start positions & max number of vehicles
% - add acceleration limits to plot for linear model