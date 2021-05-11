% Default config for a single vehicle

function cfg_vh = config_vehicle_linearLiniger(cfg)
cfg_vh = config_vehicle(cfg);

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.Linear;
cfg_vh.model_p = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt, [cfg.tempPath 'singleTrackAMax.mat']);
cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt, [cfg.tempPath 'singleTrackAMax.mat']);
end