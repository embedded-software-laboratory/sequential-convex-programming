function cfg = init_config(cfg)
% initializes given cfg
%
% Initializes handles and automatic detections
%
% Returns: initialized config struct

if isempty(cfg.scn.vhs)
    error('scenario without vehicles!')
end

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
for i = 1:length(cfg.scn.vhs)
    % detect if model is linear
    cfg.scn.vhs{i}.isControlModelLinear = isequal(cfg.scn.vhs{i}.model, @model.vehicle.Linear);
    cfg.scn.vhs{i}.isSimulationModelLinear = isequal(cfg.scn.vhs{i}.model_simulation, @model.vehicle.Linear);
    
    % Instantiate vehicle models, in partiuclar, the initialization of the
    % jacobians is crucial for gaining computation speed
    cfg.scn.vhs{i}.model = ...
        cfg.scn.vhs{i}.model(cfg.scn.vhs{i}.p.Hp, cfg.scn.vhs{i}.p.dt_controller, cfg.scn.vhs{i}.model_p);
    cfg.scn.vhs{i}.model_simulation = ...
        cfg.scn.vhs{i}.model_simulation(cfg.scn.vhs{i}.p.Hp, cfg.scn.vhs{i}.p.dt_simulation, cfg.scn.vhs{i}.model_simulation_p);
    
    if ~isfield(cfg.scn.vhs{i}.model_p, 'bounds')
        warning("Vehicle's %i model has no bounds", i);
    end
    
    % expand start states to match simulation model states
    assert(isequal(size(cfg.scn.vhs{i}.x_start), [4 1]))
    cfg.scn.vhs{i}.x_start = [cfg.scn.vhs{i}.x_start' zeros(1, cfg.scn.vhs{i}.model_simulation.nx - 4)]';
    
    %% Inputs
    % SL Acceleration: pre-compute for speed
    delta_angle = 2*pi / cfg.scn.vhs{i}.p.n_acceleration_limits;
    tmp = (1:cfg.scn.vhs{i}.p.n_acceleration_limits)' .* delta_angle;
    cfg.scn.vhs{i}.p.acceleration_cos = cos(tmp);
    cfg.scn.vhs{i}.p.acceleration_sin = sin(tmp);
end

%% Approximation
for i = 1:length(cfg.scn.vhs)
    cfg.scn.vhs{i}.approximationIsSL = ...
        cfg.scn.vhs{i}.approximation == cfg.scn.vhs{i}.approximationSL;
    cfg.scn.vhs{i}.approximationIsSCR = ...
        cfg.scn.vhs{i}.approximation == cfg.scn.vhs{i}.approximationSCR;
    
    if ~cfg.scn.vhs{i}.approximationIsSL && ~cfg.scn.vhs{i}.approximationIsSCR
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
for i = 1:length(cfg.scn.vhs)
    if cfg.scn.vhs{i}.approximationIsSCR
        cfg.scn.track_polygons = controller.gen_track_SCR.main(cfg.scn.track, cfg.scn.track_creation_scale, cfg.scn.track_SCR_epsilon_area_tolerance).polygons;
        break
    end
end
end