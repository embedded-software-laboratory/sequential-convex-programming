function cfg = scenario_models_comparison_linear(cfg)
% EXPERIMENTAL scenario to compare vehicle models:
%   single-track (controller & simulation) vs.
%   linear/single-track (controller/simulation) vs.
%   linear(controller & simulation)
cfg.scn.description = [cfg.scn.description '\nwith ST/ST vs lin/ST vs lin/lin controller/simulation'];


%% Vehicles
base_vehicle = config.base_vehicle(cfg);

% vehicle 1: single-track liniger
cfg.scn.vhs{end + 1} = config.vehicle_ST_Liniger(base_vehicle);

% vehicle 2: linear liniger controller, single-track liniger simulation
cfg.scn.vhs{end + 1} = config.vehicle_lin_Liniger_ST_Liniger(base_vehicle);

% vehicle 3: linear liniger
cfg.scn.vhs{end + 1} = config.vehicle_lin_Liniger(base_vehicle);
end