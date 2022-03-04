function sim_result = sl_vs_scr(has_changed)
    
    if (nargin==0) has_changed=false; end
    scenarios = run(true);
    scenario = scenarios(31);
    scenario.outputPath = '../results/sl_vs_scr/';
    output_file = [scenario.outputPath, 'log.mat'];
    if (has_changed) || (~isfile(output_file))
        output_file = sim.run(scenario);
    end

    sim_result = load(output_file);
%% Visualization setup
% recreate figure handles
sim_result.cfg.plot.plots_to_draw = config.config().plot.plots_to_draw;
% re-init config (so that objects & figure handles are present again)
sim_result.cfg = config.init_config(sim_result.cfg);
sim_result.cfg.plot.grayscale = true;

%% Visualization
disp('run plotting')
j = 5;
% reconstruct ws from log
ws = sim_result.log.lap{j};
for i = 1:length(sim_result.cfg.scn.vhs)
    ws.vhs{i} = sim_result.log.vehicles{i}(j);
end

i_race = 1;
sim_result.cfg.plot.plots_to_draw{i_race}.plot(sim_result.cfg, ws);

fig = sim_result.cfg.plot.plots_to_draw{i_race}.figure_handle;

legend('','','','','','','SCR','','','','SL','','','','','Location','northwest')
xlim([3.6 5.9]);
ylim([-3.2 -1.9]);
utils.set_figure_properties(fig,'paper',4);
exportgraphics(fig, [sim_result.cfg.outputPath 'slvsscr.pdf'], 'ContentType','vector');


% legend: v1,v2,track,track bound l/r,track bound l/r,track
% start/stop,trajectory 1,previous trajectory 1,vehicle 1, convex
% approximation 1, trajctory 2, previous trajectory 2, vehicle 2, SL l/r,
% SL l/r