function cfg_vh = vehicle_ST_Kloock(cfg_vh)
% adapts vehicle to Single Track Model Kloock
cfg_vh.description = [cfg_vh.description '\nwith single-track vehicle model & Kloock params'];

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model_controller = @model.vehicle.SingleTrack;
cfg_vh.modelParams_controller = model.vehicle.SingleTrack.getParamsKloockRC_1_43_WithLinigerBounds();
cfg_vh.model_simulation = cfg_vh.model_controller;
cfg_vh.modelParams_simulation = model.vehicle.SingleTrack.getParamsKloockRC_1_43_WithLinigerBounds();
end