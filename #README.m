% main file to run: 'run.m'

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