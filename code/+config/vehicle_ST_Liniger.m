function cfg_vh = vehicle_ST_Liniger(cfg_vh)
% adapts vehicle to Single Track Model Liniger
cfg_vh.description = [cfg_vh.description '\nwith single-track vehicle model & Liniger params'];

%% from Parameter Study (HockenheimShort, SL)
% found by SL parameter study
% cfg_vh.p.Q = 1; % weight for maximization of position on track
% cfg_vh.p.R = diag([30 70]); % weight for control changes over time
% cfg_vh.p.trust_region_size = 0.3; % [m] adds/subtracts to position (SL only)
% found by SCR + SL parameter study (way better lap times)
cfg_vh.p.Q = 1.5; % or even more?2.6 for maximum aggressive but close to instable; % weight for maximization of position on track
cfg_vh.p.R = diag([90 90]); % 41 for maximum aggrasion: 14.6s. 90: 15.1 % weight for control changes over time
cfg_vh.p.trust_region_size = .4; % [m] adds/subtracts to position (SL only)
% %% from Parameter Study (HockenheimShort, SCR)
% SCR has was worse lap times - why? should not be the case
cfg_vh.p.Q = 1.2; %2.6 for maximum aggressive but close to instable; % weight for maximization of position on track
cfg_vh.p.R = diag([90 90]); % 41 for maximum aggrasion: 14.6s. 90: 15.1 % weight for control changes over time

%% Model
% CAVE: model params should match across controller and simulation model
cfg_vh.model_controller_handle = @model.vehicle.SingleTrack;
cfg_vh.modelParams_controller = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
cfg_vh.model_simulation_handle = cfg_vh.model_controller_handle;
cfg_vh.modelParams_simulation = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
end