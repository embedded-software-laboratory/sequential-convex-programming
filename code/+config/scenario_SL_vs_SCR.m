function cfg = scenario_SL_vs_SCR(cfg)
% Showcasing SL vs SCR track discretization
cfg.scn.description = [cfg.scn.description '\nwith comparison of SL vs SCR track discretization controller'];

%% Vehicles
vehicle_default = config.vehicle_ST_Liniger(config.base_vehicle(cfg));

% vehicle 1: SL
cfg.scn.vhs{end + 1} = vehicle_default;
cfg.scn.vhs{1}.p.SCP_iterations = 2;

% vehicle 2: SCR
cfg.scn.vhs{end + 1} = config.vehicle_SCR(vehicle_default);