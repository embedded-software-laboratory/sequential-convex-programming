function cfg = scenario_main_vehicle_SL_vs_SCR(cfg)
% Showcasing SL vs SCR track discretization
cfg.scn.description = [cfg.scn.description '\nwith comparison of SL vs SCR track discretization controller'];

cfg.scn.is_main_vehicle_only = true;

%% Vehicles
vehicle_default = config.base_vehicle(cfg);
vehicle_default = config.vehicle_ST_Liniger(vehicle_default);
vehicle_default.p.Hp = 80; % to enable large track coverage
warning('Chosen large H_p for large track coverage. Please pause at step  ~126 to see differences of prediction over whole track')

% vehicle 1: SL
cfg.scn.vhs{end + 1} = vehicle_default;

% vehicle 2: SCR
vh = config.vehicle_SCR(vehicle_default);

% (not required here, using same model)
% scale R to account for different input sizes
% warning('(future FIXME when introducing overall scaling): remove scaling of R in scenario, when global scaling is introduced')
% warning('hard-coded scaling values (based on model bounds)')
% CAVE hard-coded values
% vh.p.R = 1/(10 -- 10)^2 * (1 -- .1)^2 * cfg.scn.vhs{end}.p.R;
cfg.scn.vhs{end + 1} = vh;
end