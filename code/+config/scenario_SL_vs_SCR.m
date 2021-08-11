function cfg = scenario_SL_vs_SCR(cfg)
% Showcasing SL vs SCR track discretization
cfg.scn.description = [cfg.scn.description '\nwith comparison of SL vs SCR track discretization controller'];

%% Vehicles
vehicle_default = config.vehicle_ST_Liniger(config.base_vehicle(cfg));

% vehicle 1: SL
vehicle_default.p.Q = 17; % weight for maximization of position on track
vehicle_default.p.R = diag([55 9]); % weight for control changes over time
vehicle_default.p.trust_region_size = 0.11; % SL only
cfg.scn.vhs{end + 1} = vehicle_default;

% vehicle 2: SCR
vehicle_default.p.Q = 1; %0.9; % weight for maximization of position on track
vehicle_default.p.R = diag([90 90]); % weight for control changes over time
cfg.scn.vhs{end + 1} = config.vehicle_SCR(vehicle_default);