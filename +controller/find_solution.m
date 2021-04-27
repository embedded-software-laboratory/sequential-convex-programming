function controller_output = find_solution(cfg, ws, i_vehicle)
% OPTIMIZER Optimize convexified solution using QP
% This script prepares all information for the problem formulation and
% solving.

timer = tic;

vh = cfg.scn.vs{i_vehicle};

x0 = ws.vs{i_vehicle}.x0; % 1 stage: current x(0)

% Decision variables
% use states x and inputs u from last timestep
X = ws.vs{i_vehicle}.x; % Hp stages: x(2), x(3), ... x(Hp-1), x(Hp), x(Hp)
U = ws.vs{i_vehicle}.u; % Hp stages: u(1), u(2), ... x(Hp-1), x(Hp)


%% Optimization Iterations
for i = 1:vh.p.iterations
    if ~vh.isModelLinear
        if i ~= 1 % if not first iteration
            % resubstitute input u from last time-step, delay optimal
            % inputs - required, as numerical discretization for non-linear
            % bicycle models requires u_{k+1} for x_{k+1}
            % FIXME why? (duplicated with main "run" loop?)
            % FIXME should ber synced with SCR, too
            U = [ws.vs{i_vehicle}.u(:,1) U(:,1:end-1)];
        end
    end
 
 
    if vh.approximationIsSCR
        % For each point of the projected trajectory, find the index
        % of the track polygon index
        track_polygon_indices = utils.find_closest_track_polygon_index(...
            X(cfg.scn.vs{i_vehicle}.model.ipos, :), cfg.scn.track_polygons, vh.p.Hp);
    else
        track_polygon_indices = NaN;
    end
    
    % FIXME this shouldn't be necessary for SCR. It's only added for quick
    % adding vehicle obstacles & blocking
    % For each point of the projected trajectory, find the index
    % of the euclidian-distance-closest track checkpoint
    checkpoint_indices = utils.find_closest_track_checkpoint_index(...
        X(cfg.scn.vs{i_vehicle}.model.ipos, :), cfg.scn.track, vh.p.Hp);
    
    %% Formulate QP
    [n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin, ...
        A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper] = ...
        controller.createQP(cfg, x0, X, U, checkpoint_indices, track_polygon_indices, i, i_vehicle, ws);
    
    %% Solve QP
    % update states x and inputs u with optimized results
    [X, U, optimization_log] = controller.solveQP(...
        cfg, cfg.scn.vs{i_vehicle}.p,...
        n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin, ...
        A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper);
    
    iterations{1, i}.x_opt = X;
    iterations{1, i}.u_opt = U;
    iterations{1, i}.optimization_log = optimization_log;
    iterations{1, i}.checkpoint_indices = checkpoint_indices;
    iterations{1, i}.track_polygon_indices = track_polygon_indices;
end

%% Output preparation 
if optimization_log.slack > 0.1
    warning('Lateral deviation unavoidable');
end

% print status
optimization_time = toc(timer);
fprintf('vehicle %i optT %6.0fms flag %i iter %3i slack %6.2f fval %6.1f\n', i_vehicle, optimization_time * 1000, optimization_log.exitflag, vh.p.iterations, optimization_log.slack, optimization_log.fval);

controller_output = struct;
controller_output.iterations = iterations;
controller_output.x_final = X;
controller_output.u_final = U;
controller_output.optimizationTime = optimization_time;
end