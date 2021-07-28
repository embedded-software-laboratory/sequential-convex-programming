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
% Some scenarios are provided in the following
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% select scneario  by changing the indexnumber %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scenario_selection = 2;

% Default scenario
scenarios(1) = config.scenario(cfg);
scenarios(2) = config.scenario_models_comparison_linear(config.scenario(cfg));


%% Run Selected Scenario
sim.run(scenarios(scenario_selection))