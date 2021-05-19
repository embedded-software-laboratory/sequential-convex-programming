function cfg = config()
cfg = struct;

% disable trace in case of warnings
warning off backtrace

%% Main Parameters
% folder of CPLEX' MATLAB connector & all necessary run files
% typical installation dir: 'C:\Program Files\IBM\ILOG\CPLEX_Studio1210\cplex\matlab\x64_win64';
cfg.env.cplex.path = 'D:\#local Apps\CPLEX_MATLAB_x64';

cfg.outputPath = ['output\', datestr(now, 'yyyy.mm.dd_HH_MM_SS'), '\'];
cfg.tempPath = 'temp\';
    
%% Race
cfg.race.n_laps = 3;            % Number of laps to be driven

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
plot.TrackPolygons(1);

% init loop plots - make sure to give unique figure numbers
cfg.plot.plots_to_draw = {
    plot.Race(10)
    plot.DashboardStatesNInputs(11)
    plot.DashboardAcceleration(12)};
end