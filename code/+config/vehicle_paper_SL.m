function cfg_vh = vehicle_paper_SL(cfg_vh)
% adapts vehicle similar to original SL paper 
%   (B. Alrifaee and J. Maczijewski, “Real-time Trajectory optimization for
%   Autonomous Vehicle Racing using Sequential Linearization,” in 2018 IEEE
%   Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483. 
%   doi: 10.1109/IVS.2018.8500634.)
cfg_vh.description = [cfg_vh.description '\nreplicating "SL paper" settings (linear models, specific weights & MPC params)'];

%% Controller: General Optimization
cfg_vh.p.SCP_iterations = 1;

cfg_vh.p.Hp = 40; % Number of prediction steps
cfg_vh.p.dt_controller = 0.15; % Size of prediction step
cfg_vh.p.dt_simulation = cfg_vh.p.dt_controller/10; % [s] size of simulation step

cfg_vh.p.S = 10; % Penalty weight for slack (was 1e30 for usage in quad objective with BotzBicycle)
cfg_vh.p.Q = 1;
cfg_vh.p.R = 0.01 * eye(2); % Penalty weight for control changes over time

cfg_vh.approximation = cfg_vh.approximationSL;

cfg_vh.x_start = [0 0 .1 0 0 0]';

%% Contoller: Miscellaneous Modelling
% Linearization (SL): size of Trust Region for position
cfg_vh.p.trust_region_size = 50; % from Janis, 1:1 scale

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model_controller_handle = @model.vehicle.Linear;
cfg_vh.modelParams_controller = model.vehicle.Linear.getParamsF1CarMaker(cfg_vh.p.dt_controller);
cfg_vh.model_simulation_handle = cfg_vh.model_controller;
cfg_vh.modelParams_simulation = cfg_vh.modelParams_controller;
end