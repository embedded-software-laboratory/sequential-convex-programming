function run(cfg)
% Main file to run the scenario with any given configuration
%
% debug despite try/catch block: `dbstop if caught error`, disable with
% `dbclear if error`
%
% Used Abbreviations
%   ws:     abbreviation of working set, the struct which acts as "RAM".
%           Contains only current status; every step is saved in log
%   cp:     abbreviation of checkpoint, the points which define the track path
%   cfg:    configuration
%   scn:    scenario
%   vs:     vehicles

% TODO unify scaling and parameters
%   SCR works preferably with hockenheim_simple?
%   SL + Botz with Hockenheim 4 (even there will start slapping, but finds
%   turn fast enough)

%% Initialization
clc
clear
timer_overall = tic;

% loading default config if no is provided
if ~exist('cfg', 'var')
    disp('Using config script as no provided as argument')
    cfg = config();
    cfg = config_scenario(cfg);
    cfg = init_config(cfg);
end

% create output dir if non-existing
if ~isfolder(cfg.outputPath); mkdir(cfg.outputPath); end

% initialize working and logging structures
step_sim = 0;
ws = init_ws(cfg);
log = init_log(ws, cfg);

% init controls
ctl_race_ongoing = true; % true until user ends or first vehicle finishes
global ctl_abort ctl_pause
init_control_keys(cfg.plot.plots_to_draw);

%% Execute Control & Simulation Loop
disp('########################')
disp('#- Starting race loop -#')

try
    while ctl_race_ongoing
        fprintf('--- Step %i ---\n', step_sim);
        step_sim = step_sim + 1;
        timer_loop = tic;

        %% Advance predicted state trajectories by one time step
        % keeping last state entry (stand still -> duplicated)
        % & last input entry to 0s
        % due to terminal constraints
        for i = 1:length(cfg.scn.vs)
            ws.vs{i}.x(:, 1:end-1) = ws.vs{i}.x(:, 2:end);
            
            %FIXME 
%             ws.vs{i}.u(:, 1:end-1) = ws.vs{i}.u(:, 2:end);
%             ws.vs{i}.u(:, end)     = [0; 0];
            ws.vs{i}.u(:, 1:end)   = ws.vs{i}.u(:, 1:end); % no shift of inputs as u1 input is needed for linearization together with x0
        end

        %% Current CP, position and lap
        % Get current checkpoints (corresponding to x0) and current laps
        for i = 1:length(cfg.scn.vs)
            % update checkpoints
            cp_curr = utils.find_closest_track_checkpoint_index(...
                ws.vs{i}.x0(cfg.scn.vs{i}.model.ipos), cfg.scn.track, 1);
            ws.vs{i}.cp_prev = ws.vs{i}.cp_curr;
            ws.vs{i}.cp_curr = cp_curr;

            % new lap: advance lap counter
            % FIXME: robust lap detection?
            if (ws.vs{i}.cp_prev ~= ws.vs{i}.cp_curr) && ...
                (ws.vs{i}.cp_prev / length(cfg.scn.track) >= 1) && ...
                (ws.vs{i}.cp_curr / length(cfg.scn.track) < 1)
                ws.vs{i}.lap_count = ws.vs{i}.lap_count + 1;
            end
        end

        % Determine relative positions of racing vehicles corresponding to
        % current checkpoints and current laps
        for i = 1:length(cfg.scn.vs)
            ws.vs{i}.pos = 1;
            for j = 1:length(cfg.scn.vs)
                if (ws.vs{i}.cp_curr < ws.vs{1,j}.cp_curr) && ...
                        (ws.vs{i}.lap_count == ws.vs{1,j}.lap_count) || ...
                        (ws.vs{i}.lap_count < ws.vs{1,j}.lap_count)
                    ws.vs{i}.pos = ws.vs{i}.pos + 1;
                end
            end
        end

        %% Determine obstacle relationship
        % Set obstacle-matrix entry to 1 if vehicle of row has to respect
        % vehicle in column as an obstacle, depending on the relativ
        % positioning on the race track.
        CP_halfTrack = length(cfg.scn.track)/2; % get checkpoint index at half of the track
        for i = 1:length(cfg.scn.vs) % ego vehicle
            for j = 1:length(cfg.scn.vs) % opposing vehicles
                if (i ~= j) && (...
                        ( (ws.vs{i}.cp_curr < ws.vs{1,j}.cp_curr) && ...
                        ( (ws.vs{1,j}.cp_curr - ws.vs{i}.cp_curr) < CP_halfTrack ) ) || ...
                        ( (ws.vs{i}.cp_curr >= ws.vs{1,j}.cp_curr) && ...
                        ( (ws.vs{i}.cp_curr - ws.vs{1,j}.cp_curr) > CP_halfTrack ) ) || ...
                        (ws.vs{i}.cp_curr == ws.vs{1,j}.cp_curr) )
                   ws.obstacleTable(i,j) = 1;
                else
                   ws.obstacleTable(i,j) = 0;
                end
            end
        end

        %% Determine Defending Relationship -> Blocking
        for i = 1:length(cfg.scn.vs) % ego vehicle
            for j = 1:length(cfg.scn.vs) % opposing vehicles
               if (i ~= j) && ...
                       ( norm(ws.vs{i}.x0(3:4)) < norm(ws.vs{1,j}.x0(3:4)) ) && ...
                       ( ws.obstacleTable(i,j) == 0 ) && ...
                       ( ( norm(ws.vs{i}.x0(1:2) - ws.vs{1,j}.x(1:2,1)) <= 0.1 ) )%|| ...
    %                    ( norm(ws.vs{i}.x0(1:2) - ws.vs{j}.x(1:2, 2)) <= 0.3 ) || ...
    %                    ( norm(ws.vs{i}.x0(1:2) - ws.vs{j}.x(1:2, 3)) <= 0.3 ) )
                   ws.blockingTable(i,j) = 1;
               else
                   ws.blockingTable(i,j) = 0;
               end
            end
        end        

        %% Controller
        for i = 1:length(cfg.scn.vs)
            % evaluate controller & save output of current vehicle to output-struct for all vehicles
            ws.vs{i}.controller_output = controller.find_solution(cfg, ws, i);

            % Write controller output (output-struct) for each vehicle to
            % (trajectory-struct)
            ws.vs{i}.x = ws.vs{i}.controller_output.x_final;
            ws.vs{i}.u = ws.vs{i}.controller_output.u_final;
        end

        %% Log
        % saving ws for vehicles seperately for easier access
        log.lap{end + 1} = rmfield(ws, 'vs');

        for i = 1:length(cfg.scn.vs)
            log.vehicles{i}(end + 1) = ws.vs{i};
        end

        %% Visualization
        % Visualization plots x0 (current state) of last time step and
        % predictions of Hp states x calculated in the current time step.
        % In other words - the visualization takes place before the
        % simulation/application of the control inputs.
        if cfg.plot.is_enabled
            for i = 1:length(cfg.plot.plots_to_draw)
                cfg.plot.plots_to_draw{i}.plot(cfg.scn, ws);
            end
            drawnow
        end

        %% Simulation: calc input response / next state
        for i = 1:length(cfg.scn.vs)        
            % TODO unify linear with non-linear models
            % if linear model
            if cfg.scn.vs{i}.isModelSimulationLinear
                % calc next timestep's state
                % equals to controller output `ws.vs{i}.x0 = ws.vs{i}.controller_output.x(:,1)`;
                ws.vs{i}.x0 = ws.vs{i}.x0 + cfg.scn.vs{i}.model_simulation.ode(ws.vs{i}.x0, ws.vs{i}.u(:,1));
            else
                ws.vs{i}.x0 = simulate_ode(ws.vs{i}, cfg.scn.vs{i});
            end
        end

        %% Execution control
        % Check if race finished
        for i = 1:length(cfg.scn.vs)
            if ws.vs{i}.lap_count >= cfg.race.n_laps
                ctl_race_ongoing = false;
            end
        end

        % enact user inputs
        if ctl_pause; disp('Pause...'); end
        while true % pseudo do..while loop
            if ctl_abort % ..stop race
                disp('Aborted');
                ctl_race_ongoing = false;
                break;
            end
            if ~ctl_pause; break; end
            pause(0.1)
        end
        fprintf('Loop time %4.0fms\n', toc(timer_loop) * 1000)
    end
catch ME
    warning('#- Error in race loop -#')
end
disp('#-  Ending race loop  -#')
disp('########################')
fprintf('Overall loop time %.2fs\n', toc(timer_overall))

%% Save 
% save all workspace variables, but not figures
disp('Saving workspace')

% remove figure handles, so they won't get saved
cfg.plot.plots_to_draw = NaN;
save([cfg.outputPath '/log.mat'])

% if error occured in race loop
if exist('ME', 'var')
    warning('Error in race loop ocurred, rethrowing:')
    rethrow(ME)
end
end


function x0_new = simulate_ode(ws_vehicle, veh_cfg)
    % solve ode for individual vehicle with calculated inputs
    % Inputs:
    %   ws_vehicle (struct): working set of individual vehicle
    %   veh_cfg (struct): vehicle configuration

    if ws_vehicle.x0(3) == 0
       ws_vehicle.x0(3) = eps; 
    end
    sim_timestep = veh_cfg.p.dt/10; % Size of time step [s]
    sim_dt = 0:sim_timestep:veh_cfg.p.dt; % Time span [s] -> n+ time points including n time steps
    sim_u = repmat(ws_vehicle.u(:,1),1,length(sim_dt)); % ^n+1 constant control inputs
    [~,sim_x0] = ode15s(...
        @(t,x)...
        veh_cfg.model_simulation.ode(...
            x, sim_u(:,round(t/sim_timestep+1))),...
        sim_dt,...
        ws_vehicle.x0,...
        odeset('RelTol',1e-8,'AbsTol',1e-8));
    x0_new = sim_x0(end,:)';
end

function ws = init_ws(cfg)
    % initialize working structure

    ws = struct;
    % vehicle-specfic working set
    for i = 1:length(cfg.scn.vs)
        % controller-specifics
        ws.vs{i}.controller_output = NaN;
        ws.vs{i}.x0 = cfg.scn.vs{i}.xStart';
        ws.vs{i}.x = repmat(cfg.scn.vs{i}.xStart', 1, cfg.scn.vs{i}.p.Hp);
        ws.vs{i}.u = repmat([0;0], 1, cfg.scn.vs{i}.p.Hp);

        % lap-specific
        cp_x0 = utils.find_closest_track_checkpoint_index(...
            ws.vs{i}.x0(cfg.scn.vs{i}.model.ipos), cfg.scn.track, 1);
        ws.vs{i}.cp_prev = cp_x0;
        ws.vs{i}.cp_curr = cp_x0;
        ws.vs{i}.lap_count = 0; % start with 0 finished laps
        ws.vs{i}.pos = 0; % ego vehicle position relative to all other vehicles
    end

    % inter-vehicle working set content
    % TODO correct description? Initialize tables with indications for each vehicle (rows) which other vehicle has to be considered as an obstacle (colums) or wich other vehicle has to be blocked (columns)
    ws.obstacleTable = zeros(length(cfg.scn.vs), length(cfg.scn.vs));
    ws.blockingTable = zeros(length(cfg.scn.vs), length(cfg.scn.vs));
end

function log = init_log(ws, cfg)
    % initialize logging structure on basis of given working set

    %% Data Logging
    % Initialize logging structure
    fields = fieldnames(ws)';
    fields(ismember(fields, 'vs')) = [];
    fields{2,1} = {};
    log.lap = struct(fields{:});

    % copy structure from ws
    fields = fieldnames(ws.vs{1})';
    fields{2,1} = {};
    for i = 1:length(cfg.scn.vs)
        log.vehicles{i} = struct(fields{:});   % Struct to save all data for each lap
    end
end

function init_control_keys(plots_to_draw)
    % initalize control keys on given plots
    global ctl_abort ctl_pause
    ctl_pause = false;
    ctl_abort = false;
    
    % Plot control: SPACE to pause, ESC to abort
    function key_press_callback(~,eventdata)
        if strcmp(eventdata.Key, 'escape')
            ctl_abort = true;
        elseif strcmp(eventdata.Key, 'space')
            ctl_pause = ~ctl_pause;
        end
    end

    % enable callbacks on every plot
    for i = 1:length(plots_to_draw)
        set(plots_to_draw{i}.figure_handle, ...
            'WindowKeyPressFcn', @key_press_callback);
    end
end