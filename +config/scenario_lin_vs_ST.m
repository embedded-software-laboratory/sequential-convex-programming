function cfg = scenario_lin_vs_ST(cfg)
% Showcasing linear vs. non-linear vehicle model controller

%% Vehicles
vehicle_default = config.base_vehicle(cfg);
warning('Chosen large H_p for large track coverage. Please pause at step  ~126 to see differences of prediction over whole track')

% vehicle 1: single-track liniger
vh = config.vehicle_ST_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vh;

% vehicle 2: linear liniger
vh = config.vehicle_lin_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vh;
end