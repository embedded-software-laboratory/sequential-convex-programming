function cfg_vh = vehicle_lin_Liniger(cfg_vh)
% adapts vehicle to linear Liniger model
cfg_vh.description = [cfg_vh.description '\nwith linear model & params from ST Liniger'];

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model_controller = @model.vehicle.Linear;
cfg_vh.modelParams_controller = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt_controller, cfg_vh.tempFileSingleTrackAMax);
cfg_vh.model_simulation = cfg_vh.model_controller;
% CAVE FIXME linear models get simulated differently --> using
% `dt_controller` instead of `dt_simulation` for now
cfg_vh.modelParams_simulation = model.vehicle.Linear.getParamsSingleTrackLiniger(cfg_vh.p.dt_controller, cfg_vh.tempFileSingleTrackAMax);
end