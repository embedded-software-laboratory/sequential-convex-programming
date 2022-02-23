function eval_lap_time()
    scenarios = run(true);

    scenarios(10).scn.vhs{1}.p.SCP_iterations = 2; % 2 iterations for SL
    
    s_to_eval = [
        scenarios(10)                                   % SL for one lap
        config.scenario_n_laps_race(scenarios(10), 2)   % SL for two laps
        scenarios(4)                                    % SCR for one lap
        config.scenario_n_laps_race(scenarios(4), 2)    % SCR for two laps
    ];
    
    
    results(length(s_to_eval)).lap_time = [];

    for i_s = 1:length(s_to_eval)
        output_file = sim.run(s_to_eval(i_s));
        sim_result = load(output_file);
        % calculate lap_time (depending on x_start and track's finish line):
        %   actually 1 lap + a fraction of a lap
        %   just using length of indices, as made sure that only 1 lap is run
        %   in scenario
        results(i_s).lap_time = length(sim_result.log.vehicles{1}) * sim_result.cfg.scn.vhs{1}.p.dt_controller;
    end

    for i_s = [2,4]
        results(i_s).lap_time = results(i_s-1).lap_time;
    end
    % t_sl_init = results(1).lap_time;
    % t_sl_flying = results(2).lap_time;
    % t_scr_init = results(3).lap_time;
    % t_scr_flying = results(4).lap_time;

    x_vals = categorical(...
        {
            't_{SL,init}'
            't_{SL,fly}'
            't_{SCR,init}'
            't_{SCR,fly}'
        }...
    );
    x_vals = reordercats(x_vals,...
        {
            't_{SL,init}'
            't_{SL,fly}'
            't_{SCR,init}'
            't_{SCR,fly}'
        }...
    );
    figure;
    bar(x_vals,[results.lap_time]);
    disp([results.lap_time]);
end