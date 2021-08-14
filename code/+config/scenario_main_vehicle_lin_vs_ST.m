function cfg = scenario_main_vehicle_lin_vs_ST(cfg)
% Showcasing linear vs. non-linear vehicle model controller overlayed
cfg.scn.description = [cfg.scn.description '\nwith comparison of lin vs ST model controller (executed on one vehicle)'];

cfg.scn.is_main_vehicle_only = true;

%% Vehicles
vehicle_default = config.base_vehicle(cfg);

% vehicle 1: single-track liniger
vh = config.vehicle_ST_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vh;

% vehicle 2: linear liniger
vh = config.vehicle_lin_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vh;
end