
%% simulate
scenarios = run(true);
output_file = sim.run(scenarios(31));

%% Visualization setup
% recreate figure handles
cfg.plot.plots_to_draw = config.config().plot.plots_to_draw;
% re-init config (so that objects & figure handles are present again)
cfg = config.init_config(cfg);
cfg.plot.grayscale = true;

%% Visualization
disp('run plotting')
j = 5;
% reconstruct ws from log
ws = log.lap{j};
for i = 1:length(cfg.scn.vhs)
    ws.vhs{i} = log.vehicles{i}(j);
end

for i = 1:length(cfg.plot.plots_to_draw)
    cfg.plot.plots_to_draw{i}.plot(cfg, ws);
end

fig = figure(10);
fig.CurrentAxes.Legend.String = {'SCR' 'SL'};
xlim([4 6]);
ylim([-3.2 -1.7]);
try
    set_figure_properties(fig,'paper',8);
catch
    disp('setting figure properties failed');
end
fig.CurrentAxes.Legend.Location = 'best';
exportgraphics(fig, [cfg.outputPath 'slvsscr.pdf'], 'ContentType','vector');


% legend: v1,v2,track,track bound l/r,track bound l/r,track
% start/stop,trajectory 1,previous trajectory 1,vehicle 1, convex
% approximation 1, trajctory 2, previous trajectory 2, vehicle 2, SL l/r,
% SL l/r