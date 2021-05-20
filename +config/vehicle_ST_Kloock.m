% Default config for a single vehicle

function cfg_vh = vehicle_ST_Kloock(cfg)
cfg_vh = config.vehicle(cfg);

% FIXME really different weight necessary?
cfg_vh.p.R = 350 * eye(2); % Penalty weight for control changes over time

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.SingleTrack;
cfg_vh.model_p = model.vehicle.SingleTrack.getParamsKloockRC_1_43();
cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.SingleTrack.getParamsKloockRC_1_43();
end