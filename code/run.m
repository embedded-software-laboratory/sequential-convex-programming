% main run file & examples how to use this software
%
%% Initial Remarks
% when running this software the first time, some simulation data needs to
%   be prepared -> don't be confused by the outputs and warnings
clc
clear
% get scenario independent cfg
cfg = config.config();
% default scenario (not runable)
scenarios(1) = config.base_scenario(cfg);

%% Scenarios  
% some preconfigured scenarios are provided in the following.
% just cascade scenario options to your like, as done below

% SCR
% one vehicle only
scenarios(4) = config.scenario_1_vehicle(scenarios(1), @(cfg_veh) config.vehicle_SCR(config.vehicle_ST_Liniger(cfg_veh)));
scenarios(6) = config.scenario_1_vehicle(scenarios(1), @(cfg_veh) config.vehicle_SCR(config.vehicle_lin_Liniger(cfg_veh)));
% ... with some (2) laps
scenarios(7) = config.scenario_n_laps_race(scenarios(4), 2);
scenarios(9) = config.scenario_n_laps_race(scenarios(6), 2);

% SL
% one vehicle only
scenarios(10) = config.scenario_1_vehicle(scenarios(1), @config.vehicle_ST_Liniger);
scenarios(12) = config.scenario_1_vehicle(scenarios(1), @config.vehicle_lin_Liniger);
% ... and with endless race
scenarios(13) = config.scenario_endless_race(scenarios(10));
scenarios(15) = config.scenario_endless_race(scenarios(12));
% ... and with reduced checkpoints
scenarios(16) = config.scenario_SL_reduce_checkpoints(scenarios(13));
scenarios(18) = config.scenario_SL_reduce_checkpoints(scenarios(15));
% actual race situations
scenarios(19) = config.scenario_race_various_vehicles(scenarios(1));

% different controller per vehicle
scenarios(21) = config.scenario_lin_vs_ST(scenarios(1));
scenarios(22) = config.scenario_SL_vs_SCR(scenarios(1));
% ... as endless race
scenarios(26) = config.scenario_endless_race(scenarios(21));
scenarios(27) = config.scenario_endless_race(scenarios(22));

% only one main vehicle, of which state various controllers are calculated
%   and displayed overlayed
scenarios(30) = config.scenario_main_vehicle_lin_vs_ST(scenarios(1));
scenarios(31) = config.scenario_main_vehicle_SL_vs_SCR(scenarios(1));

% previous paper comparion
scenarios(40) = config.scenario_paper_SL(scenarios(1));

%% Run Selected Scenario
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% select scneario  by changing the indexnumber %
%                                              %
%                             here  ||         %
%                                   vv         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if true % run only selected scenario
    output_file = sim.run(scenarios(22));
else % alternative: run all scenarios from above
        fprintf('Executing scenario %i', i);
        
        % recreate figure handles
        cfg.plot.plots_to_draw = config.config().plot.plots_to_draw;
        
        scenario = scenarios(i);
        if ~isempty(scenario.scn)
            if scenario.race.n_laps > 10 % limit race length
                scenario.race.n_laps = 2;
            end
            output_file = sim.run(scenario);
        end
    end
end

% Loding saved file after finished scenario
fprintf(2, 'Loaded output file of sim, so you can use individual evaluation scripts or replay, if desired\n')
load(output_file)