% Default config for a single vehicle

function cfg_vh = config_vehicle_ST(cfg)
cfg_vh = config_vehicle(cfg);

% FIXME really different weight necessary?
cfg_vh.p.R = 350 * eye(2); % Penalty weight for control changes over time

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.SingleTrack;
cfg_vh.model_p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43();
% CAVE: to match Liniger params enable (acc. to
% https://github.com/alexliniger/MPCC/blob/master/Matlab/getMPC_vars.m)
cfg_vh.p.TR_motorTorque = 1;

cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43();
end