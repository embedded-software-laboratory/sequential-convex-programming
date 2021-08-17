function output_file = run(cfg)
% Main file to run the scenario with any given configuration
%
% debug despite try/catch block: `dbstop if caught error`, disable with
% `dbclear if error`
%
% Used Abbreviations
%   ws:     abbreviation of working set, the struct which acts as "RAM".
%           Contains only current status; every step is saved in log
%               all variables are assumed to be simulation data, exceptions
%               are declared (e.g. controller internal data with postfix
%               "_controller")
%   cp:     abbreviation of checkpoint, the points which define the track path
%   cfg:    configuration
%   scn:    scenario
%   vhs:    vehicles

%% Initialization
timer_overall = tic;

if verLessThan('matlab', '9.10')
    warning('This is an old MATLAB version. Not all features of the software will work')
end

% initialization of objects etc.
cfg = config.init_config(cfg);

% initialize working and logging structures
step_sim = 0;
ws = sim.init_ws(cfg);
log = sim.init_log(ws, cfg);

% init controls
ctl_race_ongoing = true; % true until user ends or first vehicle finishes
global ctl_abort ctl_pause
init_control_keys(cfg.plot.plots_to_draw);

%% Execute Control & Simulation Loop
disp('########################')
disp('#- Starting race loop -#')

try
    while ctl_race_ongoing
        fprintf('------------------------- Step %i -------------------------\n', step_sim);
        step_sim = step_sim + 1;
        timer_loop = tic;
   

        %% Controller Execution
        for i = 1:length(cfg.scn.vhs)
            %% Measuring
            % "measure reality" (simulation) to models
            %   only required for linear control combined with single-track
            %   simulation models
            if cfg.scn.vhs{i}.isControlModelLinear && ~cfg.scn.vhs{i}.isSimulationModelLinear 
                % convert  states from Single Track to Linear
                ws.vhs{i}.x_0_controller = model.vehicle.state_st2lin(ws.vhs{i}.x_0);
            else
                ws.vhs{i}.x_0_controller = ws.vhs{i}.x_0;
            end

            %% Controller Execution
            % prepare vehicle working set
            vhs = cell(1, length(ws.vhs));
            for k = 1:length(ws.vhs)
                vhs{k}.x_0 = ws.vhs{k}.x_0_controller;
                vhs{k}.X_opt = ws.vhs{k}.X_controller;
                if k == i
                    vhs{k}.U_opt = ws.vhs{k}.U_controller;
                end
                vhs{k}.cp_curr = ws.vhs{k}.cp_curr;
            end

            % save *raw* output
            ws.vhs{i}.controller_output = controller.run_SCP(cfg,...
                vhs, ws.obstacleTable, ws.blockingTable, i);

            % save payload (predicted trajectories) for easier access
            ws.vhs{i}.X_controller = ws.vhs{i}.controller_output(end).X_opt;
            ws.vhs{i}.U_controller = ws.vhs{i}.controller_output(end).U_opt;
            ws.vhs{i}.u_1 = ws.vhs{i}.U_controller(:, 1);
        end
        

        %% Log
        if cfg.log.level >= cfg.log.LOG
            % saving ws for vehicles seperately for easier access
            log.lap{end + 1} = rmfield(ws, 'vhs');

            for i = 1:length(cfg.scn.vhs)
                log.vehicles{i}(end + 1) = ws.vhs{i};
            end
        end

        %% Visualization
        % Visualization plots x_0 (current state) of last time step and
        % predictions of Hp states x calculated in the current time step.
        % In other words - the visualization takes place before the
        % simulation/application of the control inputs.
        if cfg.plot.is_enabled
            for i = 1:length(cfg.plot.plots_to_draw)
                cfg.plot.plots_to_draw{i}.plot(cfg, ws);
            end
            drawnow
        end

        %% Simulation
        % compute input response / advance to next state/x_0
        for i = 1:length(cfg.scn.vhs)
            % if only simulation for main vehicle ("equipped" with
            % multiple controllers), copy it's x_0 to other vehicles
            % (which are only to compare controllers at each step)
            if cfg.scn.is_main_vehicle_only && i > 1
                % copy trajectory of main vehicle to other controllers
                if ~cfg.scn.vhs{i}.isSimulationModelLinear
                    ws.vhs{i}.x_0 = ws.vhs{1}.x_0;
                else
                    ws.vhs{i}.x_0 = model.vehicle.state_st2lin(ws.vhs{1}.x_0);
                end
                
                % don't simulate
                continue
            end
            
            isCtrLin = cfg.scn.vhs{i}.isControlModelLinear;
            isSimLin = cfg.scn.vhs{i}.isSimulationModelLinear;
            % three cases:
            % Controller Model  Simulation Model
            %      Linear            Linear
            %      Linear         Single Track
            %   Single Track      Single Track
            if isSimLin && isCtrLin
                % equals to controller output `ws.vhs{i}.x_0 = ws.vhs{i}.controller_output.x(:,1)`;
                ws.vhs{i}.x_0 = ws.vhs{i}.x_0 + cfg.scn.vhs{i}.model_simulation.ode(ws.vhs{i}.x_0, ws.vhs{i}.u_1);
            elseif ~isSimLin && isCtrLin
                ws.vhs{i}.x_0 = sim.simulate_ode(ws.vhs{i}.x_0, ws.vhs{i}.u_1, cfg.scn.vhs{i});
            elseif ~isSimLin && ~isCtrLin
                ws.vhs{i}.x_0 = sim.simulate_ode(ws.vhs{i}.x_0, ws.vhs{i}.u_1, cfg.scn.vhs{i});
            else
                % should never happen: why choose worse simulation model than controller?
                error('Combination of controller and simulation vehicle model types not supported')
            end
        end
        
        % shift controller data acc. to simulation advance
        ws = controller.shift_prev_data(length(cfg.scn.vhs), ws);
        
        % updates adminstrative data of working set to current simulation
        % state
        ws = sim.update_administrative_data(cfg, ws);

        %% Execution control
        % Check if race finished (every vehicle reached n_laps finish line
        ctl_race_ongoing = false;
        for i = 1:length(cfg.scn.vhs)
            if ws.vhs{i}.lap_count < cfg.race.n_laps
                ctl_race_ongoing = true;
                break;
            end
        end

        % enact user inputs
        if cfg.plot.is_enabled
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
if cfg.log.level >= cfg.log.LOG
    % save all workspace variables, but not figures
    disp('Saving workspace')

    % remove figure handles, so they won't get saved
    cfg.plot.plots_to_draw = NaN;
    output_file = [cfg.outputPath '/log.mat'];
    save(output_file)
end

% if error occured in race loop
if exist('ME', 'var')
    warning('Error in race loop ocurred, rethrowing:')
    rethrow(ME)
end

%% Evaluation
% Run example evaluation scipts
evaluation.plot_t_opts()
evaluation.plot_track_with_speed()

% export all currently open figures (especially for CodeOcean)
utils.exportAllFigures(cfg);
fprintf(2, 'Result files (graphics, figures, logs) were saved in "%s"\n', cfg.outputPath)
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