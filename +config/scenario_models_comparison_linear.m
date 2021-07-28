function cfg = scenario_models_comparison_linear(cfg)
% EXPERIMENTAL scenario to compare vehicle models:
%   single-track (controller & simulation) vs.
%   linear/single-track (controller/simulation) vs.
%   linear(controller & simulation)


%% Vehicles
vehicle_default = config.vehicle(cfg);

vehicle_ = config.vehicle_ST_Liniger(vehicle_default);

% vehicle 1: single-track liniger
cfg.scn.vhs{end + 1} = vehicle_;

% vehicle 2: linear liniger controller, single-track liniger simulation
vehicle_ = config.vehicle_lin_Liniger(vehicle_default);
vehicle_.model_simulation = @model.vehicle.SingleTrack;
vehicle_.model_simulation_p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
cfg.scn.vhs{end + 1} = vehicle_;

% vehicle 3: linear liniger
vehicle_ = config.vehicle_lin_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vehicle_;
end