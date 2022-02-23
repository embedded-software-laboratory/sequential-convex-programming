%% Replay Simulation
%% How to
%   1. load a "log.mat" from the output directory into workspace
%   2. run script

% recreate figure handles
cfg.plot.plots_to_draw = config.config().plot.plots_to_draw;
% re-init config (so that objects & figure handles are present again)
cfg = config.init_config(cfg);

%% Visualization
disp('run plotting')
warning('replaying at real-time speed of vehicle 1 (other vehicles may have other discretization time dt and thus a woring speed)')

for j = 1:length(log.lap)
    tic
    % reconstruct ws from log
    ws = log.lap{j};
    for i = 1:length(cfg.scn.vhs)
        ws.vhs{i} = log.vehicles{i}(j);
    end
    
    for i = 1:length(cfg.plot.plots_to_draw)
        cfg.plot.plots_to_draw{i}.plot(cfg, ws);
    end
    drawnow
    
    if j == 1
        fprintf(2, 'start replay with press of button\n')
        pause
    end
    
    % pause to simulate real time
    % CAVE replaying at real-time of vehicle 1
    delta = cfg.scn.vhs{1}.p.dt_controller - toc;
    if delta > 0
        pause(delta);
    elseif j > 1
        warning('Replay is currently not real-time')
    end
end

disp('replay finished')