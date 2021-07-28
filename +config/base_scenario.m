function cfg = base_scenario(cfg)
% basic components of a scenario
cfg.scn.obstacles = {};
cfg.scn.vhs = {};

%% General
cfg.scn.Dsafe = 'CircleImpr'; % Chose either 'Circle' or 'Ellipse' or 'CircleImpr' or 'EllipseImpr' 
cfg.race.n_laps = 3;            % Number of laps to be driven

% only simulation for main vehicle (vehicle number 1)
%   remaining vehicles are only for comparison of overlayed controller outputs
%   main vehicle should have the most sophisticated simulation vehicle
%   model, other vehicles' simulation models aren't used
cfg.scn.is_main_vehicle_only = false;

%% Track
% e.g. HockenheimShort, testCircuitE, testCircuitLiniger
cfg.scn.track_handle = @model.track.HockenheimShort;
% TODO SCR only works with hockenheim_simple?
cfg.scn.track_SCR_epsilon_area_tolerance = .05;

%% Vehicles
% x_start [pos_x pox_y v_x v_y] will be initialized to match model states
% CAVE here you need to add vehicles
%   ensure that the start position of the vehicles is on the first
%   lap regarding the track center-checkpoints (find examples in scenario
%   files)
end