function cfg = scenario_race_various_vehicles(cfg)
% Showcasing SL vs SCR track discretization
cfg.scn.description = [cfg.scn.description '\nwith a race with obstacle and vehicle avoidance with various vehicles/controllers'];

%% Vehicles
vehicle_default = config.vehicle_avoidance(config.vehicle_ST_Liniger(config.base_vehicle(cfg)));
% make less aggressive
vehicle_default.p.trust_region_size = vehicle_default.p.trust_region_size * 0.7;
vehicle_default.p.Q = vehicle_default.p.Q * 0.5;

% vehicle 1: SL
vehicle_1 = vehicle_default;
vehicle_1.x_start = vehicle_1.x_starts(:, 8);
% enforce overtaking situations by reducing max velocity
vehicle_1.modelParams_controller.bounds(2, 3) = 1.5;
cfg.scn.vhs{end + 1} = vehicle_1;

% vehicle 2: SCR
vehicle_default.x_start = vehicle_default.x_starts(:, 7);
cfg.scn.vhs{end + 1} = vehicle_default;
vehicle_default.x_start = vehicle_default.x_starts(:, 6);
cfg.scn.vhs{end + 1} = vehicle_default;
% vehicle_default.x_start = vehicle_default.x_starts(:, 5);
% cfg.scn.vhs{end + 1} = vehicle_default;
% vehicle_default.x_start = vehicle_default.x_starts(:, 4);
% cfg.scn.vhs{end + 1} = vehicle_default;
% vehicle_default.x_start = vehicle_default.x_starts(:, 3);
% cfg.scn.vhs{end + 1} = vehicle_default;
% vehicle_default.x_start = vehicle_default.x_starts(:, 2);
% cfg.scn.vhs{end + 1} = vehicle_default;
% vehicle_default.x_start = vehicle_default.x_starts(:, 1);
% cfg.scn.vhs{end + 1} = vehicle_default;