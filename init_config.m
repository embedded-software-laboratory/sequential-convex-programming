function cfg = init_config(cfg)
% initializes given cfg
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
% Instantiate vehicle models, in partiuclar, the initialization of the
% jacobians is crucial for gaining computation speed
for i = 1:length(cfg.scn.vs)
    cfg.scn.vs{i}.model = ...
        cfg.scn.vs{i}.model(cfg.scn.vs{i}.p);
    cfg.scn.vs{i}.model_simulation = ...
        cfg.scn.vs{i}.model_simulation(cfg.scn.vs{i}.p);
end

%% Track Polygon Creation (for SCR)
% if any vehicle uses SCR controller
for i = 1:length(cfg.scn.vs)
    if isequal(cfg.scn.vs{i}.p.controller, @controller.SCR.find_solution)
        cfg.scn.track_polygons = controller.SCR.generate_track_polygons.main(cfg.scn.track).polygons;
        break
    end
end
end