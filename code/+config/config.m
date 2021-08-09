function cfg = config()
cfg = struct;

% disable trace in case of warnings
warning off backtrace

%% Main Parameters
% folder of CPLEX' MATLAB connector & all necessary run files
% typical installation dir: 'C:/Program Files/IBM/ILOG/CPLEX_Studio1210/cplex/matlab/x64_win64';
cfg.env.cplex.path = 'D:/#local Apps/CPLEX_MATLAB_x64';

cfg.startTimeStr = datestr(now, 'yyyy.mm.dd_HH_MM_SS');

% paths (must have trailing slash)
if usejava('desktop') % if run graphically
    cfg.outputPath = ['../results/', cfg.startTimeStr, '/'];
    cfg.dataPath = '../data/';
else % if on CodeOcean (or headless, in general)
    cfg.outputPath = '../results/';
    cfg.dataPath = '../data/';
end
% create output & temp dir if non-existing
if ~isfolder(cfg.outputPath); mkdir(cfg.outputPath); end
if ~isfolder(cfg.dataPath); mkdir(cfg.dataPath); end

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

% fixed plots drawn at initialization
% CAVE: hard-coded in init, here only for proper figure order
plots.TrackPolygons(1);

% init loop plots - make sure to give unique figure numbers
cfg.plot.plots_to_draw = {
    plots.Race(10)
    plots.Dashboard(11)
    plots.DashboardAcceleration(12)};
end