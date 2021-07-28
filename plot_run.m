% rewatch run
% load "log.mat", then run script

% FIXME recreate figure handles
cfg.plot.plots_to_draw = {
    plot.Race(1)
    plot.DashboardStatesNInputs(2)
    plot.DashboardAcceleration(3)};

%% Visualization
disp('run plotting')

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

    % pause to simulate real time
    % FIXME what if different prediction step sizes?
    delta = cfg.scn.vhs{1}.p.dt - toc;
    if delta > 0
        pause(delta);
    else
        warning('Replay is currently not real-time')
    end 
end

disp('run finished')