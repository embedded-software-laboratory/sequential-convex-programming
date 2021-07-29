function cfg_vh = vehicle_lin_Liniger(cfg_vh)
% adapts vehicle to linear Liniger model

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.Linear;
cfg_vh.model_p = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt, cfg_vh.tempFileSingleTrackAMax);
cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt, cfg_vh.tempFileSingleTrackAMax);
end