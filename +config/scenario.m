%   Adds scenario specific settings to cfg: track,  
%   vehicles based on config_vehicle_, obstacles, etc.
%
%   Default values of e.g. vehicles can be overwritten. In doing so,
%   ensure that the start position of the vehicles is on the first
%   lap regarding the track center-checkpoints.

function cfg = scenario(cfg)
cfg.scn.obstacles = {};
cfg.scn.vs = {};

%% General
cfg.scn.Dsafe = 'CircleImpr'; % Chose either 'Circle' or 'Ellipse' or 'CircleImpr' or 'EllipseImpr' 

%% Track
% e.g. HockenheimShort, testCircuitE
cfg.scn.track_handle = @model.track.testCircuitLiniger;
% TODO SCR only works with hockenheim_simple?
cfg.scn.track_SCR_epsilon_area_tolerance = .05;

%% Vehicles
% x_start [pos_x pox_y v_x v_y] will be initialized to match model states

% Vehicle
vehicle_ = config.vehicle_ST(cfg);
vehicle_.x_start = [0 0 .1 0]';
cfg.scn.vs{end + 1} = vehicle_;

% vehicle 2
vehicle_ = config.vehicle_SCR(cfg);
vehicle_.x_start = [0 -0.05 0.1 0]';
% vehicle_.p.TR_velX = 1.5 * vehicle_.p.TR_velX; % increase max velocity for Bicycle
% vehicle_.p.a_max = 17; % decrease accel for SCR
cfg.scn.vs{end + 1} = vehicle_;

% vehicle 3
vehicle_ = config.vehicle(cfg);
vehicle_.x_start = [0.9 0.05 0.1 0]';
cfg.scn.vs{end + 1} = vehicle_;

% % vehicle 4
% vehicle_ = config.vehicle_ST_Kloock(cfg);
% vehicle_.x_start = [1.3 -0.05 0.1 0]';
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle 5
% vehicle_ = config_vehicle
% vehicle_.x_start = [1.7 0.05 0.1 0]';
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle 6
% vehicle_ = config_vehicle
% vehicle_.x_start = [2.1 -0.05 0.1 0]';
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle 7
% vehicle_ = config_vehicle
% vehicle_.x_start = [2.5 0.05 0.1 0]';
% cfg.scn.vs{end + 1} = vehicle_;
% 
% % vehicle 8
% vehicle_ = config_vehicle
% vehicle_.x_start = [2.8 -0.05 0.1 0]';
% cfg.scn.vs{end + 1} = vehicle_;
end