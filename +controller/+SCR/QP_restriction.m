function [x, U, optimization_log] = QP_restriction(cfg, x0, x, u, track_polygon_indices, i_vehicle)
% QP Create and solve convexified, restricted problem

p = cfg.scn.vs{i_vehicle}.p;
model = cfg.scn.vs{i_vehicle}.model;
% TODO unify
isLinear = cfg.scn.vs{i_vehicle}.isModelLinear;

assert(size(track_polygon_indices, 1) == p.Hp);
assert(size(track_polygon_indices, 2) == 1);

%% Index mapping
idx        = reshape((1:model.ns * p.Hp), model.ns, p.Hp)';
idx_x      = idx(:, model.ix);
idx_pos    = idx(:, model.ipos);
idx_u      = idx(:, model.iu);
idx_slack  = model.ns * p.Hp + 1; % in case of SL: lateral

%% Problem size
n_vars = model.ns * p.Hp + 1; % number of variables: (states + inputs) * prediction steps + slack variable
n_eqns = model.nx * p.Hp; % number of equations: state equations * prediction steps
if isLinear; n_eqns = n_eqns + 2; end % terminating conditions (v_x, v_y)
% if ~isLinear; n_eqns = n_eqns + 3; end % terminating conditions (v_x, v_y, dt/dyaw)

n_ineq = 0;
% TODO ease and merge with SL?
for k = 1:p.Hp
    n_ineq = n_ineq + length(cfg.scn.track_polygons(track_polygon_indices(k)).b);
end
if isLinear
    n_ineq = n_ineq + p.n_acceleration_limits * p.Hp; % acceleration bounds at every time step
end

objective_lin   = zeros(n_vars, 1);
objective_quad  = zeros(n_vars, n_vars);
A_eq            = zeros(n_eqns, n_vars);
b_eq            = zeros(n_eqns, 1);
A_ineq          = zeros(n_ineq, n_vars);
b_ineq          = zeros(n_ineq, 1);
bound_lower     =  -inf(n_vars, 1);
bound_upper     =   inf(n_vars, 1);

%% Equalities
n_rows = 0; % for double check against defined problem size above

% Get linearized and discretized system matrices
% x_k+1 = Ad * x_k + Bd * u_k+1 + Ed
[Ad, Bd, Ed] = model.calculatePredictionMatrices([x0 x(:, 1:end-1)], u);

% k = 1
n_rows = n_rows(end) + (1:model.nx);
A_eq(n_rows, idx_x(1,:)) = eye(model.nx);                                % Coeff. x_k+1
A_eq(n_rows, idx_u(1,:)) = -Bd(:,:,1);                 % Coeff. u_k+1
b_eq(n_rows) = Ad(:,:,1) * x0 + Ed(:,1);   % Ad * x_k + Ed with x_k = x0 (current state)

% k = 2:Hp
for k = 2:p.Hp
    n_rows = n_rows(end) + (1:model.nx);
    A_eq(n_rows, idx_x(k,:)) = eye(model.nx);                  % Coeff. x_k+1
    A_eq(n_rows, idx_x(k-1,:)) = -Ad(:,:,k);  % Coeff. x_k
    A_eq(n_rows, idx_u(k,:)) = -Bd(:,:,k);    % Coeff. u_k+1
    b_eq(n_rows) = Ed(:,k);                   % Ed
end

%% Terminating conditions
if isLinear
    % v_{x,y}{Hp} = 0 (b_eq already inited to 0)
    n_rows = n_rows(end) + (1:2);
    A_eq(n_rows, idx_x(k, 3:4)) = eye(2);
else    
%     % v_{x,y}{Hp} = 0 (b_eq already inited to 0)
%     n_rows = n_rows(end) + (1:2);
%     A_eq(n_rows, idx_x(k, 3:4)) = eye(2);
% 
%     % dt/dyaw at Hp = 0 (b_eq already inited to 0)
%     n_rows = n_rows(end) + 1;
%     A_eq(n_rows, idx_x(k, 6)) = 1;
end

assert(n_rows(end) == n_eqns);

%% Inequalities
n_rows = 0;
for k = 1:p.Hp
    %% Track limits
    track_polygon = cfg.scn.track_polygons(track_polygon_indices(k));
    n_rows = n_rows(end) + (1:length(track_polygon.b));
    A_ineq(n_rows, idx_slack) = -1;
    A_ineq(n_rows, idx_pos(k,:)) = track_polygon.A;
	
    b_ineq(n_rows) = track_polygon.b;
    
    % TODO 1/x restrict non-linear similar to linear (as we are in SCR)
    if isLinear
        %% Acceleration limits    
        for i = 1:p.n_acceleration_limits
            n_rows = n_rows(end) + (1);
            % simple circle, linearized
            A_ineq(n_rows, idx_u(k,:)) = [cos(2*pi*i/p.n_acceleration_limits) sin(2*pi*i/p.n_acceleration_limits)];
            b_ineq(n_rows) = p.a_max;
        end
    end
end

assert(n_rows(end) == n_ineq);

%% Objective
% Maximize position along track
objective_lin(idx_pos(p.Hp,:)) = -p.Q * cfg.scn.track_polygons(track_polygon_indices(p.Hp)).forward_direction;

% Minimize control change over time (36)
objective_quad(idx_u(p.Hp, :), idx_u(p.Hp, :)) = p.R;
for i = 1:p.Hp - 1
    objective_quad(idx_u(  i, :),idx_u(  i, :)) = 2 * p.R;  
    objective_quad(idx_u(  i, :),idx_u(i+1, :)) = -p.R; 
    objective_quad(idx_u(i+1, :),idx_u(  i, :)) = -p.R;
end
objective_quad(idx_u(1,:),idx_u(1,:)) = p.R;

% Minimize slack var (parameter q in paper?)
if isLinear
    objective_lin(idx_slack) = p.S;
else
    % TODO: Bassam - warum hier slack bei quad nötig? 
    objective_quad(idx_slack, idx_slack) = p.S;
end

%% Bounds
% Slack Var must be positive
bound_lower(idx_slack) = 0;

% TODO 1/x restrict non-linear similar to linear (as we are in SCR)
if isLinear
    % Bounded acceleration
    bound_upper(idx_u(:)) =  p.a_max;
    bound_lower(idx_u(:)) = -p.a_max;
else
    % Bounded inputs
    bound_upper(idx_u(:,1)) =  p.TR_steeringAngle;
    bound_upper(idx_u(:,2)) =  p.TR_motorTorque;
    bound_lower(idx_u(:,1)) = -p.TR_steeringAngle;
    bound_lower(idx_u(:,2)) = -p.TR_motorTorque;

    bound_upper(idx_u(end,1)) =  0.15 * p.TR_steeringAngle;
    bound_upper(idx_u(end,2)) =  0.15 * p.TR_motorTorque;
    bound_lower(idx_u(end,1)) = -0.15 * p.TR_steeringAngle;
    bound_lower(idx_u(end,2)) = -0.15 * p.TR_motorTorque;

    % Bounded states (trust region for change in position) - kinetic
    for k=1:p.Hp
        bound_upper(idx_pos(k,:)) = x(1:2,k) + p.TR_pos;
        bound_lower(idx_pos(k,:)) = x(1:2,k) - p.TR_pos;
    end

    % Bounded states (trust region for change in velocities) - kinematic
    bound_upper(idx_x(:,3)) = p.TR_velX;
    bound_upper(idx_x(:,4)) = p.TR_velY;
    bound_upper(idx_x(:,6)) = p.TR_velW;
    bound_lower(idx_x(:,3)) = 0.005;
    bound_lower(idx_x(:,4)) = -p.TR_velY;
    bound_lower(idx_x(:,6)) = -p.TR_velW;

    % Bounded states for last prediction step instead of term. constr.
    bound_upper(idx_x(end,3)) = 0.01;
    bound_upper(idx_x(end,4)) = 0.01;
    bound_upper(idx_x(end,6)) = 0.02;
    bound_lower(idx_x(end,3)) = 0.005;
    bound_lower(idx_x(end,4)) = -0.01;
    bound_lower(idx_x(end,6)) = -0.02;
end

%% Solve QP
[x, U, optimization_log] = controller.QP_solver(...
    cfg, p, n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin,...
    A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper);
end