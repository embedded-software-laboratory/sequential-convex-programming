function controller_output = run_SCP(cfg,...
    vhs, obstacleTable, blockingTable, i_vehicle)
% run RTI/SCP

%% Ease access
vh = cfg.scn.vhs{i_vehicle};

% (Fixed) initial state
x_0 = vhs{i_vehicle}.x_0; % 1 stage: last cycle's x(1)

% Decision variables (recover from last controller output)
% warm start for solver: use states x and inputs u from last time-step
X_opt = vhs{i_vehicle}.X_opt; % Hp stages: last cycle's       x(2), x(3), ..., x(Hp-1), x(Hp), x(Hp)
U_opt = vhs{i_vehicle}.U_opt; % Hp stages: last cycle's u(1), u(2), u(3), ..., u(Hp-1), u(Hp)

% Preallocate for speed
controller_output = struct('X_opt', 'U_opt', 'log_opt',...
    'checkpoint_indices', 'track_polygon_indices', 't_opt');
controller_output.X_opt(vh.p.SCP_iterations) = NaN;


%% Optimization Iterations
for i = 1:vh.p.SCP_iterations
    timer = tic;
 
    if vh.approximationIsSCR
        % For each point of the projected trajectory, find the index
        % of the track polygon index
        track_polygons = cfg.scn.track_polygons;
        track_polygon_indices = controller.track_SCR.find_closest_polygon_indices(...
            X_opt(vh.model_controller.idx_pos, :), cfg.scn.track_polygons, vh.p.Hp);
    else
        track_polygons = NaN;
        track_polygon_indices = NaN;
    end
    
    % not necessary for SCR, but vehicle obstacles & blocking
    %   For each point of the projected trajectory, find the index
    %   of the euclidian-distance-closest track checkpoint
    checkpoint_indices = controller.track_SL.find_closest_checkpoint_indices(...
        X_opt(vh.model_controller.idx_pos, :), cfg.scn.track_center, vh.p.Hp);
    
    %% Formulate QP
    [n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin, ...
        A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper] = ...
            controller.create_QP(...
                vh, x_0, X_opt, U_opt,...
                cfg.scn.track, checkpoint_indices,...
                track_polygons, track_polygon_indices,...
                i, i_vehicle,...
                obstacleTable, blockingTable, vhs);
    
    %% Solve QP
    % update states x and inputs u with optimized results
    [X_opt, U_opt, log_solver] =...
            controller.solve_QP(...
                cfg.env.cplex.is_available, ...
                n_vars, idx_x, idx_u, idx_slack,...
                objective_quad, objective_lin, ...
                A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper);
    
    %% Save Output
    controller_output(i).X_opt = X_opt;
    controller_output(i).U_opt = U_opt;
    controller_output(i).log_solver = log_solver;
    controller_output(i).checkpoint_indices = checkpoint_indices;
    controller_output(i).track_polygon_indices = track_polygon_indices;
    controller_output(i).t_opt = toc(timer);
end

%% Status
if log_solver.slack > 0.1
    warning('Lateral deviation unavoidable');
end

fprintf('vehicle %i t_opt %6.0fms flag %i iter %3i slack %6.2f fval %6.1f\n',...
    i_vehicle, sum([controller_output.t_opt]) * 1000,...
    log_solver.exitflag, vh.p.SCP_iterations, log_solver.slack,...
    log_solver.fval);
end