function cfg = init_config(cfg)
% initializes given cfg
%
% Initializes handles and automatic detections
%
% Returns: initialized config struct

%% CPLEX Detection
% if CPLEX is in path: use it
if exist(cfg.env.cplex.path, 'dir')
    addpath(cfg.env.cplex.path);
    if exist('cplexqp', 'file') == 6
        disp('Using CPLEX for solving QP')
        cfg.env.cplex.is_available = true;
    else
        fprintf('Using MATLAB for solving QP as CPLEX files in path "%s" not existing\n', cfg.env.cplex.path)
        cfg.env.cplex.is_available = false;
    end
else
    fprintf('Using MATLAB for solving QP as CPLEX path "%s" is not existing\n', cfg.env.cplex.path)
    cfg.env.cplex.is_available = false;
end

%% Vehicle Models
for i = 1:length(cfg.scn.vs)
    % detect if model is linear
    cfg.scn.vs{i}.isModelLinear = isequal(cfg.scn.vs{i}.model, @model.vehicle.Linear);
    cfg.scn.vs{i}.isModelSimulationLinear = isequal(cfg.scn.vs{i}.model_simulation, @model.vehicle.Linear);
    
    % Instantiate vehicle models, in partiuclar, the initialization of the
    % jacobians is crucial for gaining computation speed
    cfg.scn.vs{i}.model = ...
        cfg.scn.vs{i}.model(cfg.scn.vs{i}.p.Hp, cfg.scn.vs{i}.p.dt, cfg.scn.vs{i}.model_p);
    cfg.scn.vs{i}.model_simulation = ...
        cfg.scn.vs{i}.model_simulation(cfg.scn.vs{i}.p.Hp, cfg.scn.vs{i}.p.dt, cfg.scn.vs{i}.model_simulation_p);
    
    % expand start states to match model states
    assert(isequal(size(cfg.scn.vs{i}.x_start), [4 1]))
    cfg.scn.vs{i}.x_start = [cfg.scn.vs{i}.x_start' zeros(1, cfg.scn.vs{i}.model.nx - 4)]';
    
    %% Inputs
    % SL Acceleration: pre-compute for speed
    delta_angle = 2*pi / cfg.scn.vs{i}.p.n_acceleration_limits;
    tmp = (1:cfg.scn.vs{i}.p.n_acceleration_limits)' .* delta_angle;
    cfg.scn.vs{i}.p.acceleration_cos = cos(tmp);
    cfg.scn.vs{i}.p.acceleration_sin = sin(tmp);
end

%% Approximation
for i = 1:length(cfg.scn.vs)
    cfg.scn.vs{i}.approximationIsSL = ...
        cfg.scn.vs{i}.approximation == cfg.scn.vs{i}.approximationSL;
    cfg.scn.vs{i}.approximationIsSCR = ...
        cfg.scn.vs{i}.approximation == cfg.scn.vs{i}.approximationSCR;
    
    if ~cfg.scn.vs{i}.approximationIsSL && ~cfg.scn.vs{i}.approximationIsSCR
        throw ME('No valid approximation chosen')
    end
end

%% Track
[cfg.scn.track, cfg.scn.track_creation_scale] = cfg.scn.track_handle();
% Precompute for speed
cfg.scn.track_center = [cfg.scn.track.center];

% plot track, includes prerun of track SCR
plot.TrackPolygons(1).plot(cfg.scn.track, cfg.scn.track_creation_scale, cfg.scn.track_SCR_epsilon_area_tolerance);

% Polygon Creation (for SCR)
% if any vehicle uses SCR controller
for i = 1:length(cfg.scn.vs)
    if cfg.scn.vs{i}.approximationIsSCR
        cfg.scn.track_polygons = controller.SCR.generate_track_polygons.main(cfg.scn.track, cfg.scn.track_creation_scale, cfg.scn.track_SCR_epsilon_area_tolerance).polygons;
        break
    end
end
end