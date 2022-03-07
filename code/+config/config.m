function cfg = config()
cfg = struct;

% disable trace in case of warnings
warning("Disabling warning's backtracing")
warning off backtrace

%% Main Parameters
% folder of CPLEX' MATLAB connector & all necessary run files
% cfg.env.cplex.path = 'D:/#local Apps/CPLEX_MATLAB_x64';
% typical installation dir:
cfg.env.cplex.path = 'C:/Program Files/IBM/ILOG/CPLEX_Studio1210/cplex/matlab/x64_win64';


%% CPLEX Detection
% if CPLEX is in path: use it
if exist(cfg.env.cplex.path, 'dir')
    addpath(cfg.env.cplex.path);
    if exist('cplexqp', 'file') == 6
        disp('Using CPLEX for solving QP')
        cfg.env.cplex.is_available = true;
    else
        fprintf("Using MATLAB for solving QP as CPLEX files in path '%s' not existing\nMATLAB's quadprog could fail when compared to CPLEX\n", cfg.env.cplex.path)
        cfg.env.cplex.is_available = false;
    end
else
    fprintf("Using MATLAB for solving QP as CPLEX path '%s' is not existing\nMATLAB's quadprog could fail when compared to CPLEX\n", cfg.env.cplex.path)
    cfg.env.cplex.is_available = false;
end

% if use MATLAB's quadprog, check if toolbox is available
if ~cfg.env.cplex.is_available && ~utils.isToolboxAvailable('Optimization Toolbox')
    error("neither CPLEX nor MATLAB's 'Optimization Toolbox' is available - install any of those");
end




cfg.startTimeStr = datestr(now, 'yyyy.mm.dd_HH_MM_SS');

% paths (must have trailing slash)
if usejava('desktop') % if run graphically
    cfg.outputPath = ['../results/', cfg.startTimeStr, '/'];
else % if on CodeOcean (or headless, in general)
    cfg.outputPath = '../results/';
end
cfg.dataPath = '../data/';
cfg.tmpPath = '../results/tmp/';

%% Logging
cfg.log.DEBUG   = 4;
cfg.log.LOG     = 3;
cfg.log.INFO    = 2;
cfg.log.WARN    = 1;
cfg.log.ERROR   = 0;

cfg.log.level   = cfg.log.DEBUG; % log level: choose from above

%% Plotting
cfg.plot.is_enabled = true;
cfg.plot.has_accelerations = true;
cfg.plot.has_focus_on_vehicle = false;
cfg.plot.grayscale = false;

% fixed plots drawn at initialization
% CAVE: hard-coded in init, here only for proper figure order
plots.TrackPolygons(1);

% init loop plots - make sure to give unique figure numbers
cfg.plot.plots_to_draw = {
    plots.Race(10)
    plots.Dashboard(11)
    plots.DashboardAcceleration(12)};
end