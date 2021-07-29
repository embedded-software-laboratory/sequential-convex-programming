% examples how to use this software
%
%% Initial Remarks
% when running this software the first time, some simulation data needs to
%   be prepared, don't be confused by the outputs and warnings
clc
clear
% get scenario independent cfg
cfg = config.config();

%% Scenarios
% just cascade scenario options to your like

% Default scenario (not runable)
scenarios(1) = config.base_scenario(cfg);

% one vehicle only
scenarios(10) = config.scenario_endless_race(config.scenario_1_vehicle(config.base_scenario(cfg), @config.vehicle_ST_Liniger));
scenarios(11) = config.scenario_endless_race(config.scenario_1_vehicle(config.base_scenario(cfg), @config.vehicle_lin_Liniger_ST_Liniger));
scenarios(12) = config.scenario_endless_race(config.scenario_1_vehicle(config.base_scenario(cfg), @config.vehicle_lin_Liniger));

% different controller per vehicle
scenarios(20) = config.scenario_models_comparison_linear(config.base_scenario(cfg));
scenarios(21) = config.scenario_endless_race(config.scenario_lin_vs_ST(config.base_scenario(cfg)));

% only one main vehicle, of which state various controllers are calculated
%   and displayed overlayed
scenarios(30) = config.scenario_main_vehicle_lin_vs_ST(config.base_scenario(cfg));
scenarios(31) = config.scenario_main_vehicle_SL_vs_SCR(config.base_scenario(cfg));

% previous paper comparion
scenarios(41) = config.scenario_paper_SL(config.base_scenario(cfg));

%% Run Selected Scenario
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% select scneario  by changing the indexnumber %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if true
    output_file = sim.run(scenarios(11));
else % alternative: run all scenarios from above
    for i = 1:length(scenarios) %#ok<UNRCH>
        scenario = scenarios(i);
        if scenario
            if scenario.race.n_laps > 10 % limit race length
                scenario.race.n_laps = 10;
            end
            output_file = sim.run(scenario);
        end
    end
end

%% Evaluation
% Loding saved file after finished scenario
disp('Loaded output file of sim, so you can use evaluation scripts')
load(output_file)

% Run example evaluation scipts
evaluation.plot_t_opts()
evaluation.plot_track_with_speed()
