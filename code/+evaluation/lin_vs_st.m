function sim_result = lin_vs_st(has_changed)
    
    if (nargin==0)
        has_changed=false;
    end
    scenarios = run(true);
    scenario = scenarios(30);
    scenario.outputPath = '../results/tmp/lin_vs_st/';
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
j = 71;
% reconstruct ws from log
ws = sim_result.log.lap{j};
for i = 1:length(sim_result.cfg.scn.vhs)
    ws.vhs{i} = sim_result.log.vehicles{i}(j);
end

fig = figure(202);
clf;
plots.Race(202).plot_track(sim_result.cfg.scn.track);
hold on
n_vehicles = numel(sim_result.cfg.scn.vhs);
c = [0.1; 0.4] .* [1 1 1];
for i_vehicle = 1:n_vehicles
    %% Planned trajectory
    plot(ws.vhs{i_vehicle}.X_controller(1,:) ...
        ,ws.vhs{i_vehicle}.X_controller(2,:) ...
        ,'.-','color',c(i_vehicle, :),'MarkerSize',7 ...
    );

    %% Vehicle Box
    vehLength = sim_result.cfg.scn.vhs{i_vehicle}.lengthVal;
    vehWidth = sim_result.cfg.scn.vhs{i_vehicle}.widthVal;
    if length(ws.vhs{i_vehicle}.x_0_controller) <= 4 % if vehicle control model is linear
    % Vehicle box (is old version: get vehicle
    % direction from current vehicle velocity vector)
        if ws.vhs{i_vehicle}.x_0_controller(3:4) ~= [0;0]
            dist = ws.vhs{i_vehicle}.x_0_controller(3:4);
        else
            dist = [1;0];
        end
        dist = dist / norm(dist);
    else
        dist = [cos(ws.vhs{i_vehicle}.x_0_controller(5));sin(ws.vhs{i_vehicle}.x_0_controller(5))]; % yaw angle
    end
    R = [dist [-dist(2); dist(1)]];
    vehicleRectangle = R * [vehLength/2 0;0 vehWidth/2] * [1 1 -1 -1;1 -1 -1 1] + repmat(ws.vhs{i_vehicle}.x_0_controller(1:2),1,4);
    fill(vehicleRectangle(1,:),vehicleRectangle(2,:),c(i_vehicle, :));
end
    
% i_race = 1;
% sim_result.cfg.plot.plots_to_draw{i_race}.plot(sim_result.cfg, ws);
% fig = sim_result.cfg.plot.plots_to_draw{i_race}.figure_handle;

% legend('','','','','','','Single-Track Model','','','','','Linear Model','','','','','Location','southeast')
legend('','','','','Single-Track Model','','Linear Model','','Location','southeast')
xlim([-1.3 2.2]);
ylim([-3.5 -0.8]);
utils.set_figure_properties(fig,'paper',6);
exportgraphics(fig, [sim_result.cfg.outputPath 'lin_vs_st.pdf'], 'ContentType','vector');


% legend: v1,v2,track,track bound l/r,track bound l/r,track
% start/stop,trajectory 1,previous trajectory 1,vehicle 1, convex
% approximation 1, trajctory 2, previous trajectory 2, vehicle 2, SL l/r,
% SL l/r