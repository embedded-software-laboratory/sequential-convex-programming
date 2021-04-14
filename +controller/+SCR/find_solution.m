function controller_output = find_solution(cfg, ws, i_vehicle) 
% was x0, controller_output_previous)

vehicle_p = cfg.scn.vs{i_vehicle}.p;


x0 = ws.vs{i_vehicle}.x0; % 1 stage: current x(0)

% use states x and inputs u from last timestep
x = ws.vs{i_vehicle}.x; % Hp stages: x(2), x(3), ... x(Hp-1), x(Hp), x(Hp)
u = ws.vs{i_vehicle}.u; % Hp stages: u(1), u(2), ... x(Hp-1), x(Hp)

timer = tic;

for i = 1:vehicle_p.iterations
    % For each trajectory point, find the closest track polygon.
    track_polygon_indices = nan(vehicle_p.Hp, 1);
	
    for k = 1:vehicle_p.Hp
        position = x(cfg.scn.vs{i_vehicle}.model.ipos, k);
    
        min_signed_distance = 1e300;
        argmin_signed_distance = 0;
            
        for j = length(cfg.scn.track_polygons):-1:1
            signed_distance = max(cfg.scn.track_polygons(j).A * position - cfg.scn.track_polygons(j).b);
            if min_signed_distance > signed_distance
                min_signed_distance = signed_distance;
                argmin_signed_distance = j;
            end
        end
        track_polygon_indices(k) = argmin_signed_distance;
    end
    
    % Formulate and solve the restricted trajectory optimization problem.
    [x, u, optimization_log] = controller.SCR.QP_restriction(cfg, x0, x, u, track_polygon_indices, i_vehicle);
    
    iterations{1, i}.x_opt = x;
    iterations{1, i}.u_opt = u;
    iterations{1, i}.optimization_log = optimization_log;
    iterations{1, i}.track_polygon_indices = track_polygon_indices;
end

% print status
optimization_time = toc(timer);
fprintf('vehicle %i optT %6.0fms flag %i iter %3i slack %6.2f fval %6.1f\n', i_vehicle, optimization_time * 1000, optimization_log.exitflag, vehicle_p.iterations, optimization_log.slack, optimization_log.fval);

controller_output = struct;
controller_output.iterations = iterations;
controller_output.x_final = x;
controller_output.u_final = u;
controller_output.optimizationTime = optimization_time;
end