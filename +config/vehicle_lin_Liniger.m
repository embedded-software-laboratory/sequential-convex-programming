function cfg_vh = vehicle_lin_Liniger(cfg_vh)
% adapts vehicle to linear Liniger model

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model_controller = @model.vehicle.Linear;
cfg_vh.modelParams_controller = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt_controller, cfg_vh.tempFileSingleTrackAMax);
cfg_vh.model_simulation = cfg_vh.model_controller;
cfg_vh.modelParams_simulation = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt_simulation, cfg_vh.tempFileSingleTrackAMax);
end