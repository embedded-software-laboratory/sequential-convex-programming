function [x_0, X_controller_start] = compute_ST_flying_initialization()
        filepath = "../results/tmp/x_init_flying.mat";
        try
            f = load(filepath); % loads x_0 and X_controller_start
            x_0 = f.x_0;
            X_controller_start = f.X_controller_start;
        catch
            scenarios = run(true);
            scenario = scenarios(4); % scr
            scenario.outputPath = '../results/tmp/';
            n_laps = 2;
            scenario = config.scenario_n_laps_race(scenario, n_laps);
            output_file = sim.run(scenario);
            sim_result = load(output_file);
            i_last_1 = find([sim_result.log.vehicles{1}.lap_count] == 0,1, 'last');
            X_controller_start = sim_result.log.vehicles{1}(i_last_1).X_controller;
            X_controller_start = X_controller_start + [0;0;0;0;2*pi;0];
            trackWidth = 0.25;
            x_0 = sim_result.log.vehicles{1}(i_last_1).x_0;
            x_0(1,1) = 0.1;
            x_0(2,1) = trackWidth/2;
            x_0(4,1) = 0;
            x_0(5,1) = 0;
            x_0(6,1) = 0;
            % v_long = x(3);
            % v_lat  = x(4);
            % yaw    = x(5);
            % dyaw   = x(6);
            save(filepath, ...
                'x_0', 'X_controller_start' ...
            );
        end
end