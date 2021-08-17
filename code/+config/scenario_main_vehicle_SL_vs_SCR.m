function cfg = scenario_main_vehicle_SL_vs_SCR(cfg)
% Showcasing SL vs SCR track discretization
cfg.scn.description = [cfg.scn.description '\nwith comparison of SL vs SCR track discretization controller (executed on one vehicle)'];

cfg.scn.is_main_vehicle_only = true;

%% Vehicles
vehicle_default = config.vehicle_ST_Liniger(config.base_vehicle(cfg));

% vehicle 1: SCR
cfg.scn.vhs{end + 1} = config.vehicle_SCR(vehicle_default);

% vehicle 2: SL
vehicle_default.p.trust_region_size = vehicle_default.p.trust_region_size * 0.5;
cfg.scn.vhs{end + 1} = vehicle_default;