function cfg_vh = vehicle_paperSL(cfg_vh)
% adapts vehicle similar to SL paper (B. Alrifaee and J. Maczijewski, “Real-time Trajectory optimization for Autonomous Vehicle Racing using Sequential Linearization,” in 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483. doi: 10.1109/IVS.2018.8500634.)

%% Controller: General Optimization
cfg_vh.p.iterations = 1;

cfg_vh.p.Hp = 40; % Number of prediction steps
cfg_vh.p.dt = 0.15; % Size of prediction step

cfg_vh.p.S = 10; % Penalty weight for slack (was 1e30 for usage in quad objective with BotzBicycle)
cfg_vh.p.R = 0.01 * eye(2); % Penalty weight for control changes over time


%% Contoller: Miscellaneous Modelling
% Linearization (SL): size of Trust Region for position
cfg_vh.p.trust_region_size = 50; % from Janis, 1:1 scale

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.Linear;
cfg_vh.model_p = model.vehicle.Linear.getParamsF1CarMaker(cfg_vh.p.dt);
cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.Linear.getParamsF1CarMaker(cfg_vh.p.dt);
end