function cfg = base_scenario(cfg)
% base configuration for any scenario
%   contains all possible options
cfg.scn.description = 'base scenario';

cfg.scn.obstacles = {};
cfg.scn.vhs = {};

%% General
cfg.scn.Dsafe = 'CircleImpr'; % Chose either 'Circle' or 'Ellipse' or 'CircleImpr' or 'EllipseImpr' 
cfg.race.n_laps = 1;            % Number of laps to be driven

% only simulation for main vehicle (vehicle number 1)
%   remaining vehicles are only for comparison of overlayed controller outputs
%   main vehicle should have the most sophisticated simulation vehicle
%   model, other vehicles' simulation models aren't used
cfg.scn.is_main_vehicle_only = false;

%% Track
% e.g. HockenheimShort, testCircuitA testOvalB testOvalD, testCircuitLiniger
cfg.scn.track_handle = @model.track.HockenheimShort; 
% TODO SCR only works with hockenheim_simple?
cfg.scn.track_SCR_epsilon_area_tolerance = .05; % [m^2], on a 1:1 scale. will be scaled according to track creation scale

% minimum distance between checkpoints
%   zero won't change anything at given track
%   will be scaled arroding to track scale
cfg.scn.track_checkpoints_distance_min = 0;

%% Vehicles
% CAVE here you need to add vehicles
%   ensure that the start position of the vehicles is on the first
%   lap regarding the track center-checkpoints (find examples in scenario
%   files)
end