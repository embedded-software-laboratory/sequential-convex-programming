function cfg_vh = vehicle_ST_Kloock(cfg_vh)
%adapts vehicle to Single Track Model Kloock

cfg_vh.p.R = 350 * eye(2); % Penalty weight for control changes over time

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model = @model.vehicle.SingleTrack;
cfg_vh.model_p = model.vehicle.SingleTrack.getParamsKloockRC_1_43_WithLinigerBounds();
cfg_vh.model_simulation = cfg_vh.model;
cfg_vh.model_simulation_p = model.vehicle.SingleTrack.getParamsKloockRC_1_43_WithLinigerBounds();
end