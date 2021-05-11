% Default config for a single vehicle

function cfg_vh = vehicle(cfg)
cfg_vh = struct;

%% General Optimization
cfg_vh.p.iterations = 2;
cfg_vh.p.isBlockingEnabled = false;
cfg_vh.p.areObstaclesConsidered = false;

cfg_vh.p.Hp = 50; % Number of prediction steps
cfg_vh.p.dt = 0.1; % Size of prediction step

cfg_vh.p.S = 1e5; % Penalty weight for slack (was 1e30 for usage in quad objective with BotzBicycle)
cfg_vh.p.Q = 1; % Penalty weight for maximization of position on track
cfg_vh.p.R = 0.05 * eye(2); % Penalty weight for control changes over time

%% Approximation Method
% arbitrary IDs, saved for later usage
SL = 10; SCR = 20;
cfg_vh.approximationSL = SL; cfg_vh.approximationSCR = SCR;
% choose approximation
cfg_vh.approximation = SL; % 'SL' or 'SCR'

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.Linear;
cfg_vh.model_p = model.vehicle.Linear.getParamsCarMaker(cfg_vh.p.dt);
% cfg_vh.model_p = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt, [cfg.tempPath 'singleTrackAMax.mat']);
cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.Linear.getParamsCarMaker(cfg_vh.p.dt);
% cfg_vh.model_simulation_p = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt, [cfg.tempPath 'singleTrackAMax.mat']);

%% Geometric
% xStart [pos_x pox_y v_x v_y] will be initialized to match model states
cfg_vh.x_start = [0 0 0 0]';

cfg_vh.lengthVal = 0.075; % obstacle's size measured along its direction of movement [m]
cfg_vh.widthVal = 0.045; % obstacle's size measured at right angels to its direction of movement[m]

% obstacles are modeled as rotated rectangles that can move with
% constant speed and direction.
cfg_vh.distSafe2CenterVal_1 = round(sqrt((cfg_vh.lengthVal/2)^2 + (cfg_vh.widthVal/2)^2),2,'significant');
cfg_vh.distSafe2CenterVal_2 = [0.09;0.06]; % Definition of ellipsis (two semi-axis)
% TODO why there are two defined above?
cfg_vh.distSafe2CenterVal = cfg_vh.distSafe2CenterVal_1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TODO

%% Trust region - upper and lower bounds
cfg_vh.p.TR_steeringAngle = 0.40;   % Bounded input: steering angle [rad]
cfg_vh.p.TR_motorTorque = 1; 	% Bounded input: motor torque [Nm]
% FIXME (all bounds) cfg_vh.p.TR_motorTorque = 0.08; 	% Bounded input: motor torque [Nm]
cfg_vh.p.TR_pos = 0.5;     % Bounded state: position in x and y [m]
cfg_vh.p.TR_velX = 2;       % Bounded state: velocity in x [m/s]
cfg_vh.p.TR_velY = 2;       % Bounded state: velocity in y [m/s]
cfg_vh.p.TR_velW = 2*pi;       % Bounded state: velocity in W (yaw rate) [1/s]

%% Physical vehicle properties

%% TODO sl 
% %% SL Parameters
% % if strcmp(cfg.controller.name, 'SL')
    cfg_vh.p.trust_region = 3;

    % Acceleration parameters
    cfg_vh.p.n_acceleration_limits = 16; % Number of tangents around the acceleration border
    
    
    % Empirically determined maximum accelerations in the forwards, backwards
    % and lateral directions, for varying speeds.
%     cfg_vh.p.a_lateral_max_list = interp1([0 10 43 52 200], [1 14 28 33 33], 0:0.01:120);
%     cfg_vh.p.a_forward_max_list = interp1([0 10 20 35 52 79 83 200], [2 13 18 18 15 6 1 1], 0:0.01:120);
%     cfg_vh.p.a_backward_max_list = interp1([0 30 40 52 76 200], [11 13 24 30 40 40], 0:0.01:120);
%     cfg_vh.p.a_max = max([
%         cfg_vh.p.a_backward_max_list
%         cfg_vh.p.a_forward_max_list
%         cfg_vh.p.a_lateral_max_list]);
%     cfg_vh.p.v_idx = @(v) min(12001, max(1, round(100*v + 1)));
% %% SCR Parameters
% elseif strcmp(cfg.controller.name, 'SCR')
    % Acceleration parameters
    cfg_vh.p.n_acceleration_limits = 16; % Number of tangents around the acceleration border
    cfg_vh.p.a_max = 22;
% end
end