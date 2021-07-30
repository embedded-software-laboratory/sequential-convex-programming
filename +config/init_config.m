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
    cfg.scn.vhs{i}.isControlModelLinear = isequal(cfg.scn.vhs{i}.model_controller, @model.vehicle.Linear);
    cfg.scn.vhs{i}.isSimulationModelLinear = isequal(cfg.scn.vhs{i}.model_simulation, @model.vehicle.Linear);
    
    % Instantiate vehicle models
    cfg.scn.vhs{i}.model_controller = ...
        cfg.scn.vhs{i}.model_controller(cfg.scn.vhs{i}.p.Hp, cfg.scn.vhs{i}.p.dt_controller, cfg.scn.vhs{i}.modelParams_controller);
    % initialization of the jacobians for linearization is crucial for
    %   gaining computational speed in case of controllers
    if ~cfg.scn.vhs{i}.isControlModelLinear
        cfg.scn.vhs{i}.model_controller.precomputeJacobian();
    end
    
    cfg.scn.vhs{i}.model_simulation = ...
        cfg.scn.vhs{i}.model_simulation(cfg.scn.vhs{i}.p.Hp, cfg.scn.vhs{i}.p.dt_simulation, cfg.scn.vhs{i}.modelParams_simulation);
    
    if ~isfield(cfg.scn.vhs{i}.modelParams_controller, 'bounds')
        warning("Vehicle's %i controller model has no bounds (for linear models it doesn't matter)", i);
    end
    if ~isfield(cfg.scn.vhs{i}.modelParams_simulation, 'bounds')
        warning("Vehicle's %i simulation model has no bounds (for linear models it doesn't matter)", i);
    end
    
    % reduce start states to match simulation model states
    assert(isequal(size(cfg.scn.vhs{i}.x_start), [6 1]))
    cfg.scn.vhs{i}.x_start = cfg.scn.vhs{i}.x_start(1:cfg.scn.vhs{i}.model_simulation.nx);
    
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
% Reduce checkpoints
cfg.scn.track = controller.track_SL.reduce_checkpoints(cfg.scn.track, cfg.scn.track_checkpoints_distance_min, cfg.scn.track_creation_scale);
% Preallocate for speed
cfg.scn.track_center = [cfg.scn.track.center];

% plot track, includes prerun of track SCR
plot.TrackPolygons(1).plot(cfg.scn.track, cfg.scn.track_creation_scale, cfg.scn.track_SCR_epsilon_area_tolerance);

% Polygon Creation (for SCR)
% if any vehicle uses SCR controller
for i = 1:length(cfg.scn.vhs)
    if cfg.scn.vhs{i}.approximationIsSCR
        cfg.scn.track_polygons = controller.track_SCR.main(cfg.scn.track, cfg.scn.track_creation_scale, cfg.scn.track_SCR_epsilon_area_tolerance).polygons;
        break
    end
end
end