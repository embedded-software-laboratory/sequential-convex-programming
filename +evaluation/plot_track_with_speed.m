%% Plot simulation times of different vehicles
%% How to
%   1. load a "log.mat" from the output directory into workspace
%   2. run script

colors = utils.getRwthColors(100);

%% Plot track
f = figure(1020);
clf(f)
plots.Race(1020).plot_track(cfg.scn.track);
hold on
set(f, 'Name', "Vehicle 1's Velocity on Track");

%% Plot Driven Paths
%% (1) one Vehicle
x_0 = [log.vehicles{1}.x_0];
X = x_0(1, :);
Y = x_0(2, :);
velX = x_0(3, :);
velY = x_0(4, :);
vel = sqrt(velX.^2 + velY.^2);

plot(X, Y, 'Color', colors(1,:), 'LineWidth', 2)

% velocity-dependent coloring, inlcuding legend
surface('XData', [X; X],             ... % N.B.  XYZC Data must have at least 2 cols
        'YData', [Y; Y],             ...
        'ZData', [zeros(size(X)); zeros(size(X))], ...
        'CData', [vel; vel],             ...
        'FaceColor', 'none',        ...
        'EdgeColor', 'interp',      ...
        'Marker', 'none',...
        'LineWidth', 2);
colormap(jet(64))
%caxis([0 3]); % set limit
c = colorbar;
c.TickLabelInterpreter = 'latex';
c.FontSize = 11;
c.Label.String = '[m/s]';
c.Label.Interpreter = 'latex';
c.Label.FontSize = 11;


%% (4) two Vehicles no Block (velX-veh2:175, step 137)
%     scenario = twoVehScenario();
%     
%     nbStates2 = length(log.vehicles{1,2}.lapLog{1,1}) + 1;
%     X2 = nan(1,nbStates2);
%     Y2 = nan(1,nbStates2);
%     for i = 1:nbStates2
%        X2(i) = log.vehicles{1,2}.currLapLog(i).x_0(1);
%        Y2(i) = log.vehicles{1,2}.currLapLog(i).x_0(2);
%     end
%     plot(X2,Y2,'Color', color100(2,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,2}.currLapLog(137).x_0, log.vehicles{1,2}.currLapLog(137).U_opt(1), color100(2,:))
%     hold on
%     
%     nbStates1 = length(log.vehicles{1,1}.lapLog{1,1}) + 1;
%     X1 = nan(1,nbStates1);
%     Y1 = nan(1,nbStates1);
%     for i = 1:nbStates1
%        X1(i) = log.vehicles{1,1}.currLapLog(i).x_0(1);
%        Y1(i) = log.vehicles{1,1}.currLapLog(i).x_0(2);
%     end
%     plot(X1,Y1,'Color', color100(5,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,1}.currLapLog(137).x_0, log.vehicles{1,1}.currLapLog(137).U_opt(1), color100(5,:))


%% (5) two Vehicles with Block (velX-veh2:175, step 137 + 165)
%     scenario = twoVehScenario();
%     
%     nbStates2 = length(log.vehicles{1,2}.lapLog{1,1}) + 1;
%     X2 = nan(1,nbStates2);
%     Y2 = nan(1,nbStates2);
%     for i = 1:nbStates2
%        X2(i) = log.vehicles{1,2}.currLapLog(i).x_0(1);
%        Y2(i) = log.vehicles{1,2}.currLapLog(i).x_0(2);
%     end
%     plot(X2,Y2,'Color', color100(2,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,2}.currLapLog(137).x_0, log.vehicles{1,2}.currLapLog(137).U_opt(1), color100(2,:))
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,2}.currLapLog(165).x_0, log.vehicles{1,2}.currLapLog(165).U_opt(1), color100(2,:))
%     hold on
%     
%     nbStates1 = length(log.vehicles{1,1}.lapLog{1,1}) + 1;
%     X1 = nan(1,nbStates1);
%     Y1 = nan(1,nbStates1);
%     for i = 1:nbStates1
%        X1(i) = log.vehicles{1,1}.currLapLog(i).x_0(1);
%        Y1(i) = log.vehicles{1,1}.currLapLog(i).x_0(2);
%     end
%     plot(X1,Y1,'Color', color100(5,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,1}.currLapLog(137).x_0, log.vehicles{1,1}.currLapLog(137).U_opt(1), color100(5,:))
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,1}.currLapLog(165).x_0, log.vehicles{1,1}.currLapLog(165).U_opt(1), color100(5,:))


%% (7) one-four-eight Vehicles
%     nbStates1 = length(log1.vehicles{1,1}.lapLog{1,1}) + 1;
%     X1 = nan(1,nbStates1);
%     Y1 = nan(1,nbStates1);
%     vel1 = nan(1,nbStates1);
%     for i = 1:nbStates1
%        X1(i) = log1.vehicles{1,1}.currLapLog(i).x_0(1);
%        Y1(i) = log1.vehicles{1,1}.currLapLog(i).x_0(2);
%        vel1Xi = log1.vehicles{1,1}.currLapLog(i).x_0(3);
%        vel1Yi = log1.vehicles{1,1}.currLapLog(i).x_0(4);
%        vel1(i) = sqrt(vel1Xi^2 + vel1Yi^2);
%     end
%     surface('XData', [X1; X1],             ... % N.B.  XYZC Data must have at least 2 cols
%         'YData', [Y1; Y1],             ...
%         'ZData', [zeros(size(X1));zeros(size(X1))], ...
%         'CData', [vel1; vel1],             ...
%         'FaceColor', 'none',        ...
%         'EdgeColor', 'interp',      ...
%         'Marker', 'none',...
%         'LineWidth',1.5);
%     
%     nbStates2 = length(log2.vehicles{1,1}.lapLog{1,1}) + 1;
%     X2 = nan(1,nbStates2);
%     Y2 = nan(1,nbStates2);
%     vel2 = nan(1,nbStates2);
%     for i = 1:nbStates2
%        X2(i) = log2.vehicles{1,1}.currLapLog(i).x_0(1);
%        Y2(i) = log2.vehicles{1,1}.currLapLog(i).x_0(2);
%        vel2Xi = log2.vehicles{1,1}.currLapLog(i).x_0(3);
%        vel2Yi = log2.vehicles{1,1}.currLapLog(i).x_0(4);
%        vel2(i) = sqrt(vel2Xi^2 + vel2Yi^2);
%     end
%     surface('XData', [X2; X2],             ... % N.B.  XYZC Data must have at least 2 cols
%         'YData', [Y2; Y2],             ...
%         'ZData', [zeros(size(X2));zeros(size(X2))], ...
%         'CData', [vel2; vel2],             ...
%         'FaceColor', 'none',        ...
%         'EdgeColor', 'interp',      ...
%         'Marker', 'none',...
%         'LineWidth',1.5);
%     
%     colormap(jet(64))
%     caxis([0 3]);
%     c = colorbar;
%     c.TickLabelInterpreter = 'latex';
%     c.FontSize = 11;
%     c.Label.String = '[m/s]';
%     c.Label.Interpreter = 'latex';
%     c.Label.FontSize = 11;