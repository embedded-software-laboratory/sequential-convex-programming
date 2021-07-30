function cfg_vh = base_vehicle(cfg)
% base configuration for any vehicle
%   contains all possible options

% required for ST to lin Liniger conversion
cfg_vh.tempFileSingleTrackAMax = [cfg.tempPath 'singleTrackAMax.mat'];

%% Controller: General Optimization
cfg_vh.p.iterations = 2;
% SL 1, SCR 2, Botz 1
cfg_vh.p.isBlockingEnabled = false;
cfg_vh.p.areObstaclesConsidered = false;

cfg_vh.p.Hp = 50; % Number of prediction steps
% SL 40, SCR 20, Botz 20, Liniger 40
cfg_vh.p.dt_controller = 0.1; % [s] size of prediction step for controller
% SL 0.15, SCR 0.5, Botz 0.1, Liniger 0.02

% simulation step size is only relevant, if a controller to transform
%   inputs from controller to simulation model is neccessary.
%   in other cases, MATLAB's ODE solvers choose step-sizes by themselves
cfg_vh.p.dt_simulation = cfg_vh.p.dt_controller/10; % [s] size of simulation step

cfg_vh.p.S = 1e5; % weight for slack
% SL 10, SCR 1e5, Botz 1e40, (Liniger 250)
cfg_vh.p.Q = 1; % weight for maximization of position on track
% SL 1, SCR 1, Botz 1, (Liniger 0.1 ... 10)
cfg_vh.p.R = 0.05; % weight for control changes over time
% SL 0.01, SCR 0.01, Botz 500, (Liniger 0.01 ... 1)


%% Contoller: Miscellaneous Modelling
% Acceleration: number of tangents around the ellipses
cfg_vh.p.n_acceleration_limits = 16;

% Linearization (SL): size of Trust Region for position
% FIXME scale trust region size with track
cfg_vh.p.trust_region_size = 0.5;
% SL 50, SCR not required, Botz 0.06

%% Controller: Approximation Method
% arbitrary IDs, saved for later usage
cfg_vh.approximationSL = 10; cfg_vh.approximationSCR = 20;
% choose approximation
cfg_vh.approximation = cfg_vh.approximationSL; % 'approximationSL' or 'approximationSL'

 
%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model_controller = @model.vehicle.Linear;
cfg_vh.modelParams_controller = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
cfg_vh.model_simulation = cfg_vh.model_controller;
cfg_vh.modelParams_simulation = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();

%% Geometric
% xStart [pos_x pox_y v_x v_y yaw dyaw/dt] will be initialized to match model states
cfg_vh.x_start = [0 0 0 0 0 0]';

% FIXME adapt to scale. Define in vehicle model
cfg_vh.lengthVal = 0.075; % obstacle's size measured along its direction of movement [m]
cfg_vh.widthVal = 0.045; % obstacle's size measured at right angels to its direction of movement[m]

% obstacles are modeled as rotated rectangles that can move with
% constant speed and direction.
cfg_vh.distSafe2CenterVal_1 = round(sqrt((cfg_vh.lengthVal/2)^2 + (cfg_vh.widthVal/2)^2),2,'significant');
cfg_vh.distSafe2CenterVal_2 = [0.09;0.06]; % Definition of ellipsis (two semi-axis)
% TODO why there are two defined above?
cfg_vh.distSafe2CenterVal = cfg_vh.distSafe2CenterVal_1;
end