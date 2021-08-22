function cfg = scenario_paper_SL(cfg)
% scenario similar to original SL paper
%   (B. Alrifaee and J. Maczijewski, “Real-time Trajectory optimization for
%   Autonomous Vehicle Racing using Sequential Linearization,” in 2018 IEEE
%   Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483. 
%   doi: 10.1109/IVS.2018.8500634.)
cfg.scn.description = [cfg.scn.description '\nreplicating "SL paper" settings'];

%% Track
cfg.scn.track_handle = @model.track.HockenheimShortCarMaker;
cfg.scn.track_SCR_epsilon_area_tolerance = .5; % [m^2]

%% Vehicles
vehicle_ = config.vehicle_paper_SL(config.base_vehicle(cfg));

vehicle_.p.Hp = 40;
vehicle_.p.dt_controller = 0.15; % [s] size of prediction step for controller
% SL 0.15, SCR 0.5, Botz 0.1, Liniger 0.02

% simulation step size is only relevant, if a controller to transform
%   inputs from controller to simulation model is neccessary.
%   in other cases, MATLAB's ODE solvers choose step-sizes by themselves
vehicle_.p.dt_simulation = vehicle_.p.dt_controller/10; % [s] size of simulation step

vehicle_.p.S = 10; % weight for slack
vehicle_.p.Q = 1; % weight for maximization of position on track
vehicle_.p.R = 0.01 * diag([10 .1]); % weight for control changes over time


%% Contoller: Miscellaneous Modelling
% Acceleration: number of tangents around the ellipses
vehicle_.p.n_acceleration_limits = 16;
vehicle_.p.trust_region_size = 50;  % [m] adds/subtracts to position

%%
cfg.scn.vhs{end + 1} = vehicle_;
end