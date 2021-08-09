function cfg_vh = vehicle_ST_Liniger(cfg_vh)
% adapts vehicle to Single Track Model Liniger
cfg_vh.description = [cfg_vh.description '\nwith single-track vehicle model & Liniger params'];

% from Parameter Study
cfg_vh.p.Q = 17; % weight for maximization of position on track
cfg_vh.p.R = [55,0; 0,22]; % weight for control changes over time
cfg_vh.p.trust_region_size = 0.11;


%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model_controller_handle = @model.vehicle.SingleTrack;
cfg_vh.modelParams_controller = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
cfg_vh.model_simulation_handle = cfg_vh.model_controller_handle;
cfg_vh.modelParams_simulation = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
end