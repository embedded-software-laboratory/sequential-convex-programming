% Default config for a single vehicle

function cfg_vehicle = config_vehicle(model)
cfg_vehicle = struct;

%% General
cfg_vehicle.p.iterations = 2;
cfg_vehicle.p.isBlockingEnabled = false;
cfg_vehicle.p.areObstaclesConsidered = false;

% arbitrary IDs, saved for later usage
SL = 10; SCR = 20;
cfg_vehicle.approximationSL = SL; cfg_vehicle.approximationSCR = SCR;
% choose approximation
cfg_vehicle.approximation = SL; % 'SL' or 'SCR'

cfg_vehicle.model = model;
cfg_vehicle.model_simulation = cfg_vehicle.model; % model used for simulation

cfg_vehicle.isModelLinear = isequal(cfg_vehicle.model, @vehicle.Linear);
cfg_vehicle.isModelSimulationLinear = isequal(cfg_vehicle.model_simulation, @vehicle.Linear);

% obstacles are modeled as rotated rectangles that can move with
% constant speed and direction.

cfg_vehicle.xStart = @() error('Not configured! Should be size (n_x, 1)');

cfg_vehicle.lengthVal = 0.075; % obstacle's size measured along its direction of movement [m]
cfg_vehicle.widthVal = 0.045; % obstacle's size measured at right angels to its direction of movement[m]

cfg_vehicle.distSafe2CenterVal_1 = round(sqrt((cfg_vehicle.lengthVal/2)^2 + (cfg_vehicle.widthVal/2)^2),2,'significant');
cfg_vehicle.distSafe2CenterVal_2 = [0.09;0.06]; % Definition of ellipsis (two semi-axis)
% TODO why there are two defined above?
cfg_vehicle.distSafe2CenterVal = cfg_vehicle.distSafe2CenterVal_1;

%% Common Parameters
cfg_vehicle.p.Hp = 50; % Number of prediction steps
cfg_vehicle.p.dt = 0.1; % Size of prediction step


% %% paramsV2 MPC properties
cfg_vehicle.p.idx_steering_angle = 1;   % input steering angle (delta)
cfg_vehicle.p.idx_motor_torque = 2;     % input motor torque

cfg_vehicle.p.S = 1e5; % Penalty weight for slack (was 1e30 for usage in quad objective with BotzBicycle)
cfg_vehicle.p.Q = 1; % Penalty weight for maximization of position on track
if cfg_vehicle.isModelLinear
    cfg_vehicle.p.R = 0.05 * eye(2); % Penalty weight for control changes over time
else
    cfg_vehicle.p.R = 350 * eye(2); % Penalty weight for control changes over time
end

%% Trust region - upper and lower bounds
cfg_vehicle.p.TR_steeringAngle = 0.40;   % Bounded input: steering angle [rad]
cfg_vehicle.p.TR_motorTorque = 0.08; 	% Bounded input: motor torque [Nm]
cfg_vehicle.p.TR_pos = 0.5;     % Bounded state: position in x and y [m]
cfg_vehicle.p.TR_velX = 2;       % Bounded state: velocity in x [m/s]
cfg_vehicle.p.TR_velY = 2;       % Bounded state: velocity in y [m/s]
cfg_vehicle.p.TR_velW = 2*pi;       % Bounded state: velocity in W (yaw rate) [1/s]

%% Physical vehicle properties
cfg_vehicle.p.vehicleModel_m = 0.041; % vehicle mass [kg]
cfg_vehicle.p.vehicleModel_Iz = 27.8e-6; % vehicle inertia [kg m^2]
cfg_vehicle.p.vehicleModel_Lf = 0.029; % front wheel to CoG [m]
cfg_vehicle.p.vehicleModel_Lr = 0.033; % rear wheel to CoG [m]
cfg_vehicle.p.vehicleModel_N = 1; % constant for rear wheel longitudinal force: gear ratio / wheel radius [1/m]

cfg_vehicle.p.tireModel_Df = 0.1884;
cfg_vehicle.p.tireModel_Cf = 1.9669;
cfg_vehicle.p.tireModel_Bf = 3.0323;
cfg_vehicle.p.tireModel_Dr = 0.1644;
cfg_vehicle.p.tireModel_Cr = 1.3709;
cfg_vehicle.p.tireModel_Br = 4.1165;

%% TODO sl 
% %% SL Parameters
% % if strcmp(cfg.controller.name, 'SL')
    cfg_vehicle.p.trust_region = 3;

    % Acceleration parameters
    cfg_vehicle.p.n_acceleration_limits = 16; % Number of tangents around the acceleration border
    cfg_vehicle.p.delta_angle = 2*pi / cfg_vehicle.p.n_acceleration_limits; % pre-compute for speed
    % Empirically determined maximum accelerations in the forwards, backwards
    % and lateral directions, for varying speeds.
    cfg_vehicle.p.a_lateral_max_list = interp1([0 10 43 52 200],[1 14 28 33 33], 0:0.01:120);
    cfg_vehicle.p.a_forward_max_list = interp1([0 10 20 35 52 79 83 200],[2 13 18 18 15 6 1 1], 0:0.01:120);
    cfg_vehicle.p.a_backward_max_list = interp1([0 30 40 52 76 200],[11 13 24 30 40 40], 0:0.01:120);
    cfg_vehicle.p.a_max = max([
        cfg_vehicle.p.a_backward_max_list
        cfg_vehicle.p.a_forward_max_list
        cfg_vehicle.p.a_lateral_max_list]);
    cfg_vehicle.p.v_idx = @(v) min(12001, max(1, round(100*v + 1)));
% %% SCR Parameters
% elseif strcmp(cfg.controller.name, 'SCR')
    % Acceleration parameters
    cfg_vehicle.p.n_acceleration_limits = 16; % Number of tangents around the acceleration border
    cfg_vehicle.p.a_max = 22;
% end
end