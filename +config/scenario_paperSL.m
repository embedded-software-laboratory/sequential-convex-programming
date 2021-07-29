% scenario similar to SL paper (B. Alrifaee and J. Maczijewski, “Real-time Trajectory optimization for Autonomous Vehicle Racing using Sequential Linearization,” in 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483. doi: 10.1109/IVS.2018.8500634.)
function cfg = scenario_paperSL(cfg)
%% Track
% e.g. HockenheimShort, testCircuitE, testCircuitLiniger
cfg.scn.track_handle = @model.track.HockenheimShortCarMaker;
cfg.scn.track_SCR_epsilon_area_tolerance = .5;

%% Vehicles
% x_start [pos_x pox_y v_x v_y] will be initialized to match model states

% Vehicle
vehicle_ = config.vehicle_paperSL(cfg);
vehicle_.x_start = [0 0 .1 0]';
cfg.scn.vhs{end + 1} = vehicle_;
end