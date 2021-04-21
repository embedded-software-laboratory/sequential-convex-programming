function controller_output = find_solution(cfg, ws, i_vehicle)
% OPTIMIZER Optimize convexified solution using QP
% This script prepares all information for the problem formulation and
% solving in the SL_QP-script.

timer = tic;

vehicle_p = cfg.scn.vs{i_vehicle}.p;
checkpoints = cfg.scn.track;

x0 = ws.vs{i_vehicle}.x0; % 1 stage: current x(0)
% use states x and inputs u from last timestep
x = ws.vs{i_vehicle}.x; % Hp stages: x(2), x(3), ... x(Hp-1), x(Hp), x(Hp)
u = ws.vs{i_vehicle}.u; % Hp stages: u(1), u(2), ... x(Hp-1), x(Hp)


%% Optimization Iterations
for i = 1:vehicle_p.iterations
    if ~cfg.scn.vs{i_vehicle}.isModelLinear
        if i ~= 1 % if not first iteration
            u = [u(:,1) u(:,1:end-1)]; % forward u? why? TODO (duplicated with run_simulation?)
        end
    end

    % For each point of the projected trajectory, find the index
    % of the euclidian-distance-closest track checkpoint
    checkpoint_indices = utils.find_closest_track_checkpoint_index(...
        x(cfg.scn.vs{i_vehicle}.model.ipos, :), checkpoints, vehicle_p.Hp);
    
    %% Formulate QP
    [n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin, ...
        A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper] = ...
        controller.SL.createQP(cfg, x0, x, u, checkpoint_indices, i, i_vehicle, ws);
    
    %% Solve QP
    % update states x and inputs u with optimized results
    [x, u, optimization_log] = controller.solveQP(...
        cfg, cfg.scn.vs{i_vehicle}.p,...
        n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin, ...
        A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper);

    
    
    iterations{1,i}.x_opt = x;
    iterations{1,i}.u_opt = u;
    iterations{1,i}.optimization_log = optimization_log;
    iterations{1,i}.checkpoint_indices = checkpoint_indices;
end

%% Output preparation
if optimization_log.slack > 0.1
    warning('Lateral deviation unavoidable');
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