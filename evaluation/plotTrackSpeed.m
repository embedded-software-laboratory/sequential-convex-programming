% Change inputs to (log1,log2) for plotting (1) and (7)
addpath('../')
colors = getRwthColors(100);

track_ = track.Hockenheim4();
left_points = [track_(:).left];
right_points = [track_(:).right];
forward_vectors = [track_.forward_vector];

n_vehicles = 1;

f = figure(1321654541);
set(f, 'Name', 'Track Velocity-Dependent');
%     set(f,'units','centimeters','OuterPosition',[0,0,14.5,15])

%% Plot track

% Draw track area
pts=[fliplr(right_points) right_points(:,end) left_points left_points(:,1)];
fill(pts(1,:), pts(2,:),[1 1 1]*.8,'EdgeAlpha',0)
hold on

% Draw track outline with extra width for the vehicle
width = 0.045 / 2;
normals = width*[0 -1;1 0]*forward_vectors;
left_points = left_points + normals;
right_points = right_points - normals;
plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)],'k','LineWidth',1);
hold on
plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)],'k','LineWidth',1);
hold on

% Start / Finish Line
plot([right_points(1,1) left_points(1,1)],[right_points(2,1) left_points(2,1)],':k')
hold on

%% Plot Driven Paths

%% (1) one Vehicle
% FIXME: restrict data input to one lap only
x0 = [log.vehicles{1}.x0];
X = x0(1, :);
Y = x0(2, :);
velX = x0(3, :);
velY = x0(4, :);
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
caxis([0 3]);
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
%        X2(i) = log.vehicles{1,2}.currLapLog(i).x0(1);
%        Y2(i) = log.vehicles{1,2}.currLapLog(i).x0(2);
%     end
%     plot(X2,Y2,'Color', color100(2,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,2}.currLapLog(137).x0, log.vehicles{1,2}.currLapLog(137).u(1), color100(2,:))
%     hold on
%     
%     nbStates1 = length(log.vehicles{1,1}.lapLog{1,1}) + 1;
%     X1 = nan(1,nbStates1);
%     Y1 = nan(1,nbStates1);
%     for i = 1:nbStates1
%        X1(i) = log.vehicles{1,1}.currLapLog(i).x0(1);
%        Y1(i) = log.vehicles{1,1}.currLapLog(i).x0(2);
%     end
%     plot(X1,Y1,'Color', color100(5,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,1}.currLapLog(137).x0, log.vehicles{1,1}.currLapLog(137).u(1), color100(5,:))


%% (5) two Vehicles with Block (velX-veh2:175, step 137 + 165)
%     scenario = twoVehScenario();
%     
%     nbStates2 = length(log.vehicles{1,2}.lapLog{1,1}) + 1;
%     X2 = nan(1,nbStates2);
%     Y2 = nan(1,nbStates2);
%     for i = 1:nbStates2
%        X2(i) = log.vehicles{1,2}.currLapLog(i).x0(1);
%        Y2(i) = log.vehicles{1,2}.currLapLog(i).x0(2);
%     end
%     plot(X2,Y2,'Color', color100(2,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,2}.currLapLog(137).x0, log.vehicles{1,2}.currLapLog(137).u(1), color100(2,:))
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,2}.currLapLog(165).x0, log.vehicles{1,2}.currLapLog(165).u(1), color100(2,:))
%     hold on
%     
%     nbStates1 = length(log.vehicles{1,1}.lapLog{1,1}) + 1;
%     X1 = nan(1,nbStates1);
%     Y1 = nan(1,nbStates1);
%     for i = 1:nbStates1
%        X1(i) = log.vehicles{1,1}.currLapLog(i).x0(1);
%        Y1(i) = log.vehicles{1,1}.currLapLog(i).x0(2);
%     end
%     plot(X1,Y1,'Color', color100(5,:),'LineWidth', 1)
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,1}.currLapLog(137).x0, log.vehicles{1,1}.currLapLog(137).u(1), color100(5,:))
%     hold on
%     plot_vehicle(scenario.vehicles{1,1}, log.vehicles{1,1}.currLapLog(165).x0, log.vehicles{1,1}.currLapLog(165).u(1), color100(5,:))


%% (7) one-four-eight Vehicles
%     nbStates1 = length(log1.vehicles{1,1}.lapLog{1,1}) + 1;
%     X1 = nan(1,nbStates1);
%     Y1 = nan(1,nbStates1);
%     vel1 = nan(1,nbStates1);
%     for i = 1:nbStates1
%        X1(i) = log1.vehicles{1,1}.currLapLog(i).x0(1);
%        Y1(i) = log1.vehicles{1,1}.currLapLog(i).x0(2);
%        vel1Xi = log1.vehicles{1,1}.currLapLog(i).x0(3);
%        vel1Yi = log1.vehicles{1,1}.currLapLog(i).x0(4);
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
%        X2(i) = log2.vehicles{1,1}.currLapLog(i).x0(1);
%        Y2(i) = log2.vehicles{1,1}.currLapLog(i).x0(2);
%        vel2Xi = log2.vehicles{1,1}.currLapLog(i).x0(3);
%        vel2Yi = log2.vehicles{1,1}.currLapLog(i).x0(4);
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


%% Final Tuning

bounds = [min([left_points right_points]');max([left_points right_points]')];    
xlim([bounds(1,1)-0.1 bounds(2,1)+0.1])
ylim([bounds(1,2)-0.1 bounds(2,2)+0.1])

daspect([1 1 1])

xlabel('X [m]','interpreter','latex','FontSize',11)
ylabel('Y [m]','interpreter','latex','FontSize',11)
set(gca,'ticklabelinterpreter','latex','FontSize',11)

%% Save
%     print -dsvg 20190318Eval_1_DrivenPaths_oneVeh

%     print -dsvg 20190318Eval_4_DrivenPaths_twoVeh
%     print -dsvg 20190318Eval_5_DrivenPaths_twoVehBlock
%     
%     print -dsvg 20190318Eval_7_DrivenPaths_1and8Veh