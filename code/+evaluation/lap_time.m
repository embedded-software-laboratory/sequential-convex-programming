function sim_result = lap_time(has_changed)
    if (nargin==0) 
        has_changed=false;
    end
    scenarios = run(true);
    scenario = scenarios(22); % race
    scenario.outputPath = '../results/t_lap/';
    output_file = [scenario.outputPath, 'log.mat'];
    if (has_changed) || (~isfile(output_file))
        % scenario = scenarios(4); % scr
        % scenario = scenarios(10); % sl
        n_laps = 2;
        scenario = config.scenario_n_laps_race(scenario, n_laps);
        
        % iterations for SL
        assert(scenario.scn.vhs{1}.approximation == scenario.scn.vhs{1}.approximationSL);
        scenario.scn.vhs{1}.p.SCP_iterations = 1;
        % trust region
        if (scenario.env.cplex.is_available)
            scenario.scn.vhs{1}.p.trust_region_size = 1.6;
        else
            scenario.scn.vhs{1}.p.trust_region_size = 0.8;
        end

            
        output_file = sim.run(scenario);
    end

    sim_result = load(output_file);

    n_vehicles = numel(sim_result.cfg.scn.vhs);
    n_laps = sim_result.cfg.race.n_laps;
    t_lap = zeros(n_laps, n_vehicles);
    
    for i_vehicle = 1:n_vehicles
        % Lap Time
        if scenario.scn.vhs{i_vehicle}.approximation == scenario.scn.vhs{i_vehicle}.approximationSL
            convexification_method = "SL";
        else
            convexification_method = "SCR";
        end
        for i_lap = 1:n_laps
            t_lap(i_lap,i_vehicle) = sum( ...
                [sim_result.log.vehicles{i_vehicle}.lap_count] == i_lap-1) ...
                * sim_result.cfg.scn.vhs{i_vehicle}.p.dt_controller;
            fprintf("Lap %i using %3s, t_lap: %f\n", ...
                i_lap, convexification_method, t_lap(i_lap,i_vehicle)...
            );
        end
    end
end