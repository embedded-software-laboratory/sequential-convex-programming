function sim_result = race_line(has_changed)
    if (nargin==0) 
        has_changed=false;
    end
    scenarios = run(true);
    scenario = scenarios(23); % race
    scenario.outputPath = '../results/race_line/';
    output_file = [scenario.outputPath, 'log.mat'];
    if (has_changed) || (~isfile(output_file))
        % scenario = scenarios(4); % scr
        % scenario = scenarios(10); % sl
        n_laps = 1;
        scenario = config.scenario_n_laps_race(scenario, n_laps);
        scenario = config.scenario_flying_start(scenario);
        
        % iterations for SL
        assert(scenario.scn.vhs{2}.approximation == scenario.scn.vhs{2}.approximationSL);
        scenario.scn.vhs{2}.p.SCP_iterations = 1;
        % trust region
        if (scenario.env.cplex.is_available)
            scenario.scn.vhs{2}.p.trust_region_size = 1.6;
        else
            scenario.scn.vhs{2}.p.trust_region_size = 0.6;
        end
        
        output_file = sim.run(scenario);
    end
    sim_result = load(output_file);
    n_vehicles = numel(sim_result.cfg.scn.vhs);

    c = [0.1; 0.4] .* [1 1 1];
    l = ["-", "--"];
    m = [".",".","+", "x"];
    
    fig = figure(200);
    clf;
    plots.Race(200).plot_track(sim_result.cfg.scn.track);
    hold on
    for i_vehicle = 1:n_vehicles
        % Path
        indices_lap_2 = [sim_result.log.vehicles{i_vehicle}.lap_count] == 0;
        % assumes 10 steps per simulation
        n_steps_sim = 10;
        assert(size(sim_result.log.vehicles{i_vehicle}(1).x_sim,2),n_steps_sim);
        indices_lap_2_sim = logical(kron(indices_lap_2, ones(1,n_steps_sim)));
%         i_last_1 = find([sim_result.log.vehicles{i_vehicle}.lap_count] == 0,1, 'last');
%         indices_lap_2(i_last_1) = 1;
        x_all = horzcat(sim_result.log.vehicles{i_vehicle}.x_sim);
        x = x_all(:,indices_lap_2_sim);
        X = x(1, :);
        Y = x(2, :);
        plot(X, Y, ...
            'Color', c(i_vehicle,:), ...
            'LineWidth', 0.5, ...
            'LineStyle', l(i_vehicle), ...
            'Marker', m(i_vehicle) ...
            , 'MarkerIndices',1:10*n_steps_sim:length(Y) ...
            , 'MarkerSize', 7 ...
        );
    end
    legend('','','','','SCR','SL');
    xlim([-1.3 5.8])
    ylim([-4.5 0.3])
    hold off
    
    fig.CurrentAxes.Legend.Location = 'southeast';
    utils.set_figure_properties(fig,'paper',6);
    exportgraphics(fig, [sim_result.cfg.outputPath '/race_line.pdf'], 'ContentType','vector');
end