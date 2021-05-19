% Default config for a single vehicle

function cfg_vh = vehicle_ST(cfg)
cfg_vh = config.vehicle(cfg);

% FIXME really different weight necessary?
cfg_vh.p.R = 350 * eye(2); % Penalty weight for control changes over time

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.SingleTrack;
cfg_vh.model_p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43();
cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43();

%% Trust region - upper and lower bounds
cfg_vh.p.TR_steeringAngle = 0.35;   % Bounded input: steering angle [rad]
% CAVE: to match Liniger params enable (acc. to
% https://github.com/alexliniger/MPCC/blob/master/Matlab/getMPC_vars.m)
% FIXME: adapt all bounds
cfg_vh.p.TR_motorTorque = 1;
% cfg_vh.p.TR_motorTorque = 0.08; 	% Bounded input: motor torque [Nm]
cfg_vh.p.TR_pos = 0.5;     % Bounded state: position in x and y [m]
cfg_vh.p.TR_velX = 4;       % Bounded state: velocity in x [m/s]
cfg_vh.p.TR_velY = 4;       % Bounded state: velocity in y [m/s]
cfg_vh.p.TR_velW = 2*pi;       % Bounded state: velocity in W (yaw rate) [1/s]
end