% Default config for a single vehicle

function cfg = config_vehicle
cfg = struct;

%% General Optimization
cfg.p.iterations = 2;
cfg.p.isBlockingEnabled = false;
cfg.p.areObstaclesConsidered = false;

cfg.p.Hp = 50; % Number of prediction steps
cfg.p.dt = 0.1; % Size of prediction step

cfg.p.S = 1e5; % Penalty weight for slack (was 1e30 for usage in quad objective with BotzBicycle)
cfg.p.Q = 1; % Penalty weight for maximization of position on track
cfg.p.R = 0.05 * eye(2); % Penalty weight for control changes over time

%% Approximation Method
% arbitrary IDs, saved for later usage
SL = 10; SCR = 20;
cfg.approximationSL = SL; cfg.approximationSCR = SCR;
% choose approximation
cfg.approximation = SL; % 'SL' or 'SCR'

%% Model
% CAVE: model params should match across controller and simulation model
cfg.model = @model.vehicle.Linear;
cfg.model_p = model.vehicle.Linear.getParamsCarMaker(cfg.p.dt);
cfg.model_simulation = cfg.model;
cfg.model_simulation_p = model.vehicle.Linear.getParamsCarMaker(cfg.p.dt);

%% Geometric
% xStart [pos_x pox_y v_x v_y] will be initialized to match model states
cfg.x_start = [0 0 0 0]';

cfg.lengthVal = 0.075; % obstacle's size measured along its direction of movement [m]
cfg.widthVal = 0.045; % obstacle's size measured at right angels to its direction of movement[m]

% obstacles are modeled as rotated rectangles that can move with
% constant speed and direction.
cfg.distSafe2CenterVal_1 = round(sqrt((cfg.lengthVal/2)^2 + (cfg.widthVal/2)^2),2,'significant');
cfg.distSafe2CenterVal_2 = [0.09;0.06]; % Definition of ellipsis (two semi-axis)
% TODO why there are two defined above?
cfg.distSafe2CenterVal = cfg.distSafe2CenterVal_1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TODO

%% Trust region - upper and lower bounds
cfg.p.TR_steeringAngle = 0.40;   % Bounded input: steering angle [rad]
cfg.p.TR_motorTorque = 0.08; 	% Bounded input: motor torque [Nm]
cfg.p.TR_pos = 0.5;     % Bounded state: position in x and y [m]
cfg.p.TR_velX = 2;       % Bounded state: velocity in x [m/s]
cfg.p.TR_velY = 2;       % Bounded state: velocity in y [m/s]
cfg.p.TR_velW = 2*pi;       % Bounded state: velocity in W (yaw rate) [1/s]

%% Physical vehicle properties

%% TODO sl 
% %% SL Parameters
% % if strcmp(cfg.controller.name, 'SL')
    cfg.p.trust_region = 3;

    % Acceleration parameters
    cfg.p.n_acceleration_limits = 16; % Number of tangents around the acceleration border
    
    % pre-compute for speed
    delta_angle = 2*pi / cfg.p.n_acceleration_limits;
    tmp = (1:cfg.p.n_acceleration_limits)' .* delta_angle;
    cfg.p.acceleration_cos = cos(tmp);
    cfg.p.acceleration_sin = sin(tmp);
    
    % Empirically determined maximum accelerations in the forwards, backwards
    % and lateral directions, for varying speeds.
    cfg.p.a_lateral_max_list = interp1([0 10 43 52 200],[1 14 28 33 33], 0:0.01:120);
    cfg.p.a_forward_max_list = interp1([0 10 20 35 52 79 83 200],[2 13 18 18 15 6 1 1], 0:0.01:120);
    cfg.p.a_backward_max_list = interp1([0 30 40 52 76 200],[11 13 24 30 40 40], 0:0.01:120);
    cfg.p.a_max = max([
        cfg.p.a_backward_max_list
        cfg.p.a_forward_max_list
        cfg.p.a_lateral_max_list]);
    cfg.p.v_idx = @(v) min(12001, max(1, round(100*v + 1)));
% %% SCR Parameters
% elseif strcmp(cfg.controller.name, 'SCR')
    % Acceleration parameters
    cfg.p.n_acceleration_limits = 16; % Number of tangents around the acceleration border
    cfg.p.a_max = 22;
% end
end