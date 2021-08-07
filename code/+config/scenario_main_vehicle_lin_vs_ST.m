function cfg = scenario_main_vehicle_lin_vs_ST(cfg)
% Showcasing linear vs. non-linear vehicle model controller overlayed
cfg.scn.description = [cfg.scn.description '\nwith comparison of lin vs ST model controller'];

cfg.scn.is_main_vehicle_only = true;

%% Vehicles
vehicle_default = config.base_vehicle(cfg);
vehicle_default.p.Hp = 150; % to enable large track coverage
warning('Chosen large H_p for large track coverage. Please pause at step  ~126 to see differences of prediction over whole track')

% vehicle 1: single-track liniger
vh = config.vehicle_ST_Liniger(vehicle_default);
cfg.scn.vhs{end + 1} = vh;

% vehicle 2: linear liniger
vh = config.vehicle_lin_Liniger(vehicle_default);

% scale R to account for different input sizes
warning('(future FIXME when introducing overall scaling): remove scaling of R in scenario, when global scaling is introduced')
warning('hard-coded scaling values (based on model bounds)')
% CAVE hard-coded values
vh.p.R = 1/(10 -- 10)^2 * (1 -- .1)^2 * cfg.scn.vhs{end}.p.R;
cfg.scn.vhs{end + 1} = vh;
end