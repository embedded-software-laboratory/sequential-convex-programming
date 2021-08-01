function cfg = scenario_paper_SL(cfg)
% scenario similar to original SL paper
%   (B. Alrifaee and J. Maczijewski, “Real-time Trajectory optimization for
%   Autonomous Vehicle Racing using Sequential Linearization,” in 2018 IEEE
%   Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483. 
%   doi: 10.1109/IVS.2018.8500634.)
cfg.scn.description = [cfg.scn.description '\nreplicationg "SL paper" settings'];

%% Track
cfg.scn.track_handle = @model.track.HockenheimShortCarMaker;
cfg.scn.track_SCR_epsilon_area_tolerance = .5;

%% Vehicles
vehicle_ = config.vehicle_paper_SL(config.base_vehicle(cfg));
cfg.scn.vhs{end + 1} = vehicle_;
end