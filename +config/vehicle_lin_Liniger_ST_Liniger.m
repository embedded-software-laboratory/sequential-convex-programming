function cfg_vh = vehicle_lin_Liniger_ST_Liniger(cfg_vh)
% adapts vehicle to linear Liniger control and single-track liniger simulation model

cfg_vh = config.vehicle_lin_Liniger(cfg_vh);
cfg_vh.model_simulation = @model.vehicle.SingleTrackWAccelerationController;
cfg_vh.modelParams_simulation = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
end