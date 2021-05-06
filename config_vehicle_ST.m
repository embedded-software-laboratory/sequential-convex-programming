% Default config for a single vehicle

function cfg = config_vehicle_ST
cfg = config_vehicle();

% FIXME really different weight necessary?
cfg.p.R = 350 * eye(2); % Penalty weight for control changes over time

%% Model
% CAVE: model params should match across controller and simulation model
cfg.model = @model.vehicle.SingleTrack;
cfg.model_p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43();
% CAVE: to match Liniger params enable (acc. to
% https://github.com/alexliniger/MPCC/blob/master/Matlab/getMPC_vars.m)
cfg.p.TR_motorTorque = 1;

cfg.model_simulation = cfg.model;
cfg.model_simulation_p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43();
end