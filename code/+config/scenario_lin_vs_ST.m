function cfg = scenario_lin_vs_ST(cfg)
% Showcasing linear vs. non-linear vehicle model controller
cfg.scn.description = [cfg.scn.description '\nwith linear vs single-track vehicle model controller'];

%% Vehicles
vehicle_default = config.base_vehicle(cfg);

% vehicle 1: single-track liniger
vh = config.vehicle_ST_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vh;

% vehicle 2: linear liniger
vh = config.vehicle_lin_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vh;
end