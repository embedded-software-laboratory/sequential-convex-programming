clc
clear

%% [CONFIG] Parameter to Change, Vehicle & Scenario
cfg_default = config.scenario_1_vehicle(...
    config.base_scenario(config.config()),...
    @config.vehicle_ST_Liniger);
% SCR lap time: 10.7, 2 laps 20.5 -> delta = 9.8
%   mit korrigierter polygon findung: lap time: 10.5
% SL lap time: 11.8, 2 laps 21.5 -> delta = 9.7

% choose SCR
cfg_default.scn.vhs{1}.approximation = 20;
% current time-lap-optimal params
cfg_vh.p.Q = 1.5; % or even more?2.6 for maximum aggressive but close to instable; % weight for maximization of position on track
cfg_vh.p.R = diag([90 90]); % 41 for maximum aggrasion: 14.6s. 90: 15.1 % weight for control changes over time
cfg_vh.p.trust_region_size = 1.6; % [m] adds/subtracts to position (SL only)

% e.g. Q R, trust_region_size
parameter_name = 'R'; % field in `scenario_default.scn.vhs{1}.p`
% adapt below in code in case of arrays to process arbitrary values like arrays
% create parameter variation
parameter_variation_factors = 1:10:90;
for i = 1:length(parameter_variation_factors)
    if strcmp(parameter_name, 'R') % for arrays
        % first tune whole R
        parameter_variation(i).parameter = parameter_variation_factors(i) .* diag([1 1]); %#ok<SAGROW>
        % then tune individual entries
%         parameter_variation(i).parameter = diag([30 parameter_variation_factors(i)]); %#ok<SAGROW>
    else % for scalar
        parameter_variation(i).parameter = parameter_variation_factors(i); %#ok<SAGROW>
    end
end

%% Parameter Iteration Loop

% make sure only running one lap
cfg_default.race.n_laps = 1;

% preallocate
results(length(parameter_variation)).lap_time = [];
results(length(parameter_variation)).parameter = [];

for i = 1:length(parameter_variation)
    % change param, store, and run sim
    results(i).parameter = parameter_variation(i).parameter;
    cfg = cfg_default;
    cfg.scn.vhs{1}.p = setfield(cfg_default.scn.vhs{1}.p, parameter_name, parameter_variation(i).parameter); %#ok<SFLD>
    try
        output_file = sim.run(cfg);

        %% store current lap time for parameter
        % load results of current simulation
        sim_result = load(output_file) %#ok<NOPTS>
        fprintf('Current parameter iteration %i with parameter %.2f\n', i, parameter_variation(i).parameter)

        % calculate lap_time (depending on x_start and track's finish line):
        %   actually 1 lap + a fraction of a lap
        %   just using length of indices, as made sure that only 1 lap is run
        %   in scenario
        results(i).lap_time = length(sim_result.log.vehicles{1}) * cfg.scn.vhs{1}.p.dt_controller;
    catch me % in case of error in simulation due to bad parameter choice
        results(i).lap_time = 0;
        disp(me)
        warning('Error in simulation, setting lap time to 0')
    end
    
    %% Plot Results
    figure_number = 1001;
    figure(figure_number)
    clf(figure_number)
    hold on
    set(gcf, 'Name', ['Parameter Study: Parameter ' parameter_name ' vs. Lap Time'])
    if isscalar(results(1).parameter) % if scalar
        scatter([results(1:i).parameter], [results(1:i).lap_time], 'filled')
        plot([results(1:i).parameter], [results(1:i).lap_time])
        xlabel(['Parameter ' parameter_name], 'Interpreter', 'none'); ylabel('Lap Time [s]');
    else % if arrays etc.: show index
        scatter(1:i, [results(1:i).lap_time], 'filled')
        plot(1:i, [results(1:i).lap_time])
        xlabel(['Index of Parameter ' parameter_name], 'Interpreter', 'none'); ylabel('Lap Time [s]');
    end
    
    drawnow
end


% export figure (in parameter study folder)
outputPathStudy = [cfg_default.outputPath '../Parameter Study/'];
if ~isfolder(outputPathStudy); mkdir(outputPathStudy); end
utils.exportFigure(figure_number, [outputPathStudy cfg_default.startTimeStr]);