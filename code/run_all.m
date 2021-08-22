% running all pre-configured scnearios from main `run`-file 

scenarios = run(true);

for i = 10:length(scenarios)
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