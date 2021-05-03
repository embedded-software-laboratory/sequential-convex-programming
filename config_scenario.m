%   Adds scenario specific settings to cfg: track,  
%   vehicles based on config_vehicle_, obstacles, etc.
%
%   Default values of e.g. vehicles can be overwritten. In doing so,
%   ensure that the start position of the vehicles is on the first
%   lap regarding the track center-checkpoints.

function cfg = config_scenario(cfg)
cfg.scn.obstacles = {};
cfg.scn.vs = {};

%% General
cfg.scn.Dsafe = 'CircleImpr'; % Chose either 'Circle' or 'Ellipse' or 'CircleImpr' or 'EllipseImpr' 

%% Track
% e.g. `@track.Hockenheim4`, `@track.hockenheim_simple`
cfg.scn.track_handle = @track.hockenheim_simple; 
% TODO SCR only works with hockenheim_simple?

%% Vehicles
% vehicle_ 1
vehicle_ = config_vehicle(@vehicle.Linear);
% TODO: automate switching number of states depending on model / model nx
% CAVE all states of model must be contained in the first states of simulation model (in same order)
% TODO check above
if vehicle_.isModelLinear
    vehicle_.xStart = [0 0 .1 0 ];
else
    vehicle_.xStart = [0 0 .1 0 0 0];
end
cfg.scn.vs{end + 1} = vehicle_;

% vehicle_ 2
% vehicle_ = config_vehicle(@vehicle.Linear);
% % TODO couple with used vehicle_ model nx
% if vehicle_.isModelLinear
%     vehicle_.xStart = [0 -0.05 0.1 0 ];
% else
%     vehicle_.xStart = [0 -0.05 0.1 0 0 0];
% end
% vehicle_.p.TR_velX = 1.5 * vehicle_.p.TR_velX; % increase max velocity for Bicycle
% vehicle_.p.a_max = 17; % decrease accel for SCR
% cfg.scn.vs{end + 1} = vehicle_;

% % vehicle_ 3
% vehicle_ = config_vehicle_();
% vehicle_.xStart = [0.9 0.05 0.1 0 0 0]; % Start position
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle_ 4
% vehicle_ = config_vehicle_();
% vehicle_.xStart = [1.3 -0.05 0.1 0 0 0]; % Start position
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle_ 5
% vehicle_ = config_vehicle_();
% vehicle_.xStart = [1.7 0.05 0.1 0 0 0]; % Start position
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle_ 6
% vehicle_ = config_vehicle_();
% vehicle_.xStart = [2.1 -0.05 0.1 0 0 0]; % Start position
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle_ 7
% vehicle_ = config_vehicle_();
% vehicle_.xStart = [2.5 0.05 0.1 0 0 0]; % Start position
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle_ 8
% vehicle_ = config_vehicle_();
% vehicle_.xStart = [2.8 -0.05 0.1 0 0 0]; % Start position
% cfg.scn.vs{end + 1} = vehicle_;
end