function [n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin,...
    A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper] = ...
    createQP(cfg, x0, x, u, checkpoint_indices, track_polygon_indices, iter, i_vehicle, ws)
% QP formulation: create and solve convexified problem
% track can be represented as SCR or SL
% SL: Sequential Linearization controller: QP approximated (linearised, thus relaxed) variant
% SCR: Sequential Convex Restriction controller: QP in a restrictive variant

% Assign for better readability/access
vh = cfg.scn.vs{i_vehicle};
p = vh.p;
checkpoints = cfg.scn.track;
if vh.approximationIsSCR; track_polygons = cfg.scn.track_polygons; end
model = cfg.scn.vs{i_vehicle}.model;
% TODO unify
isLinear = cfg.scn.vs{i_vehicle}.isModelLinear;

if vh.approximationIsSL
    % Verification if indices-matrix has only one row and Hp columns
    assert(size(checkpoint_indices, 1) == 1);
    assert(size(checkpoint_indices, 2) == p.Hp);
elseif vh.approximationIsSCR
    % Verification if indices-matrix has only one column and Hp rows
    assert(size(track_polygon_indices, 1) == p.Hp);
    assert(size(track_polygon_indices, 2) == 1);
end

%% Index mapping
idx        = reshape((1:model.ns * p.Hp), model.ns, p.Hp)';
idx_x      = idx(:, model.ix);
idx_pos    = idx(:, model.ipos);
idx_u      = idx(:, model.iu);
idx_slack  = model.ns * p.Hp + 1;

%% Problem size
n_vars = model.ns * p.Hp + 1; % number of variables: (states + inputs) * prediction steps + slack variable
n_eqns = model.nx * p.Hp; % number of equations: state equations * prediction steps
if isLinear; n_eqns = n_eqns + 2; end % terminating conditions (v_x, v_y)
% if ~isLinear; n_eqns = n_eqns + 3; end % terminating conditions (v_x, v_y, dt/dyaw)

n_ineq = 0;


if vh.approximationIsSL
    % 2 track constraints at every time step 
    n_ineq = n_ineq + 2 * p.Hp;
elseif vh.approximationIsSCR
    % n-polygon-dependent track constraints at every time step
    for k = 1:p.Hp
        n_ineq = n_ineq + length(track_polygons(track_polygon_indices(k)).b);
    end
end

if isLinear
    n_ineq = n_ineq + p.n_acceleration_limits * p.Hp; % acceleration bounds at every time step
end
if p.areObstaclesConsidered
    % at first iteration no obstacle constraints are respected
    if ~((sum(ws.obstacleTable(i_vehicle,:)) == 0) || ((sum(ws.obstacleTable(i_vehicle,:)) ~= 0) && (iter == 1)))
        n_ineq = n_ineq + sum(ws.obstacleTable(i_vehicle,:)) * p.Hp;  % relevant obstacle constraints at every time step
    end
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
    % terminal constraints are implemented as (softer) lower/upper bounds
    % on last prediction step below
end

assert(n_rows(end) == n_eqns);

%% Inequalities

% Override obstacle constraint (set to zero) if trajectory point is ahead of corresponding obstacle trajectory point
% TODO shouldn't be this reset every iteration? How are multiple obstacles handled?
isOpponentOvertaken = false;

n_rows = 0; % for double check against defined problem size above
for k = 1:p.Hp
    %% Track limits
    % assign current checkpoint for better readability
    checkpoint_c = checkpoints(checkpoint_indices(k));
    
    if vh.approximationIsSL
        % left side (27)
        n_rows = n_rows(end) + 1;
        A_ineq(n_rows, idx_slack) = -1;
        A_ineq(n_rows, idx_pos(k,:)) = checkpoint_c.normal_vector';
        b_ineq(n_rows) = checkpoint_c.normal_vector' * checkpoint_c.left;
        % right side (28)
        n_rows = n_rows(end) + 1;
        A_ineq(n_rows, idx_slack) = -1;
        A_ineq(n_rows, idx_pos(k,:)) = -checkpoint_c.normal_vector';
        b_ineq(n_rows) = -checkpoint_c.normal_vector' * checkpoint_c.right;
    elseif vh.approximationIsSCR
        track_polygon = track_polygons(track_polygon_indices(k));
        n_rows = n_rows(end) + (1:length(track_polygon.b));
        A_ineq(n_rows, idx_slack) = -1;
        A_ineq(n_rows, idx_pos(k,:)) = track_polygon.A;
    
        b_ineq(n_rows) = track_polygon.b;
        n_rows = n_rows(end);
    end
    
    if isLinear
        %% Acceleration limits
        if vh.approximationIsSL
            [Au_acc, b_acc] = controller.SL.acceleration_constraint_tangent(p, (1:p.n_acceleration_limits)', x(:, k));
            for i = 1:p.n_acceleration_limits
                n_rows = n_rows(end) + (1);
           
                A_ineq(n_rows, idx_u(k,:)) = Au_acc(i, :);
                b_ineq(n_rows) = b_acc(i);
            end
        elseif vh.approximationIsSCR
            for i = 1:p.n_acceleration_limits
                n_rows = n_rows(end) + (1);
                % simple circle, linearized
                A_ineq(n_rows, idx_u(k,:)) = [cos(2*pi*i/p.n_acceleration_limits) sin(2*pi*i/p.n_acceleration_limits)];
                b_ineq(n_rows) = p.a_max;
            end
        end
        
        
    end


    %% Obstacle constraints and override/contraction of track constraints (only in 2nd iteration, only if obstacles are to be respected)
    % TODO: why only in second iteration
    % TODO: add warning when obstacles enabled, but p.iteration == 1
    if p.areObstaclesConsidered
        
        % TODO: see below outcommented, too
        %         % Override (constrict) track constraint on one side if opponent is significantly alongside and ego has more space (on the inside)
        %         trackCenter2ego_vec = (x(1:2,k) - checkpoint_c.center)' * left_unit_vector;
        %         trackCenter2obst_vec = (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector;
        %         xOpp = trajectories.vehicles{1,2}.x(:,k);
        %         if ( norm(trackCenter2ego_vec,2) < norm(trackCenter2obst_vec,2) ) && ...
        %             ( trajectories.blockingTable(vehNr,1) == 1 ) && ...
        %             ( abs( (x(1:2,k) - xOpp(1:2))' *  checkpoint_c.forward_vector ) <= 0.075 ) && ...
        %             ( j ~= vehNr )
        %             if (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector > 0 % obstacle on left side
        %                 b_ineq(n_rows-1) = left_unit_vector' * (checkpoint_c.left - left_unit_vector * 0.045);
        %             elseif (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector < 0 % obstacle on right side
        %                 b_ineq(n_rows) = -left_unit_vector' * (checkpoint_c.right + left_unit_vector * 0.045);
        %             end
        %         end

        if (iter >= 2) && (sum(ws.obstacleTable(i_vehicle,:)) >= 1)

            % iterate over all opponents
            for j = 1:length(cfg.scn.vs)
                % Extend predicted trajectory of opponent if the opponent's prediction horizon is shorter than the own horizon
                % TODO: very simplistic approach
                if k <= size(ws.vs{1, j}.x,2) % Choose correponding trajectory point if prediciton step is within scope
                    xOpp = ws.vs{1, j}.x(:, k);
                else % If prediction step is not within scope, choose last trajectory point
                    xOpp = ws.vs{1, j}.x(:, end);
                end    

                % Respect only constraints for opponents marked as obstacles
                if ws.obstacleTable(i_vehicle,j) == 1
                    d = pdist([x(1:2,k)';xOpp(1:2)'],'euclidean'); % calculate distance
                    normal_vector = - (x(1:2,k)-xOpp(1:2))/d; % normal vector in direction from trajectory point to obstacle center
                    V_diff = x(3:4,k) - xOpp(3:4);    % velocity difference between ego and opponent

                    if strcmp(cfg.scn.Dsafe,'Circle')
                        D_vel_1 = sqrt((V_diff(1) * p.dt)^2 + (V_diff(2) * p.dt)^2);  % safety distance due to velocity of objects
                        D_size_1 = 2*cfg.scn.vs{1,j}.distSafe2CenterVal_1;       % safety distance due to object size
                        Dsafe_1 = D_vel_1 + D_size_1;
                        closest_obst_point = xOpp(1:2) - normal_vector * Dsafe_1; % intersection of safe radius and connection between trajectory point and obstacle center
                    elseif strcmp(cfg.scn.Dsafe,'Ellipse')
                        D_vel_2 = ( normal_vector' * V_diff ) * p.dt;
                        yaw_ang = x(5,k);
                        Rot_yaw = [ cos(yaw_ang) -sin(yaw_ang) ; sin(yaw_ang) cos(yaw_ang) ];
                        D_size_2 = normal_vector' * (Rot_yaw * cfg.scn.vs{1,j}.distSafe2CenterVal_2);
                        Dsafe_2 = norm( D_vel_2 + D_size_2 );
                        closest_obst_point = xOpp(1:2) - normal_vector * Dsafe_2; % intersection of safe radius and connection between trajectory point and obstacle center
                    elseif strcmp(cfg.scn.Dsafe,'CircleImpr')
                        D_vel_1 = ( normal_vector' * V_diff ) * p.dt; % Improved
    %                         D_vel_1 = 0;
                        D_size_1 = 2*cfg.scn.vs{1,j}.distSafe2CenterVal_1;
                        Dsafe_1 = D_vel_1 + D_size_1;
                        closest_obst_point = xOpp(1:2) - normal_vector * Dsafe_1;
                    elseif strcmp(cfg.scn.Dsafe,'EllipseImpr')
                        D_vel_2 = ( normal_vector' * V_diff ) * p.dt;
                        yaw_ang = xOpp(5);
                        Rot_yaw = [ cos(yaw_ang) -sin(yaw_ang) ; sin(yaw_ang) cos(yaw_ang) ];
    %                         D_size_2 = norm( (-normal_vector) .* (Rot_yaw * cfg.scn.vs{1,j}.distSafe2CenterVal_2) );
    %                         Dsafe_2 = D_vel_2 + D_size_2;
    %                         closest_obst_point = xOpp(1:2) - normal_vector * Dsafe_2;
                        safety_ellipseA = Rot_yaw * cfg.scn.vs{1,j}.distSafe2CenterVal_2;
                        closest_obst_point = xOpp(1:2) + [ safety_ellipseA(1)*(-normal_vector(1)) ; safety_ellipseA(2)*(-normal_vector(2)) ]  - normal_vector * D_vel_2;
                    end

                    % Identify if trajectory point has already passed the corresponding obstacle trajectory point
                    left = [0 -1;1 0] * checkpoint_c.forward_vector;
                    forward = checkpoint_c.forward_vector;
                    % angle between normal vector of lin. obst. constr. and track forward vector
                    ang_normal_forward = acosd(dot(normal_vector,forward)/(norm(normal_vector)*norm(forward)));
                    % angle between normal vector of lin. obst. constr. and track left vector
                    ang_normal_left = acosd(dot(normal_vector,left)/(norm(normal_vector)*norm(left)));
                    if ( (ang_normal_forward >= 90) && (ang_normal_left >= 90) ) || ...
                       ( (ang_normal_forward >= 90) && (ang_normal_left <= 90) )
                        isOpponentOvertaken = true;
                    end

                    n_rows = n_rows(end) + (1);
                    if isOpponentOvertaken % Set obstacle constraint to zero if trajectory point has passed the corresponding obstacle trajectory point in the first iteration
                        A_ineq(n_rows, idx_slack) = 0;
                        A_ineq(n_rows, idx_pos(k,:)) = [0 0];
                        b_ineq(n_rows) = 0;

                        % TODO: see above outcommented, too
                            % Override (constrict) track constraint on one side if opponent is significantly alongside and ego has more space (on the inside)
    %                         trackCenter2ego_vec = (x(1:2,k) - checkpoint_c.center)' * left_unit_vector;
    %                         trackCenter2obst_vec = (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector;
    %                         if ( norm(trackCenter2ego_vec,2) < norm(trackCenter2obst_vec,2) ) && ...
    %                             ( abs( (x(1:2,k) - xOpp(1:2))' *  checkpoint_c.forward_vector ) <= 0.075 ) && ...
    %                             ( j ~= vehNr )
    %                             if (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector > 0 % obstacle on left side
    %                                 b_ineq(n_rows-2) = left_unit_vector' * (checkpoint_c.left - left_unit_vector * 0.045);
    %                             elseif (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector < 0 % obstacle on right side
    %                                 b_ineq(n_rows-1) = -left_unit_vector' * (checkpoint_c.right + left_unit_vector * 0.045);
    %                             end
    %                         end

                    else % Normal obstacle constraint
                        A_ineq(n_rows, idx_slack) = -1;
                        A_ineq(n_rows, idx_pos(k,:)) = normal_vector';
                        b_ineq(n_rows) = normal_vector' * closest_obst_point;
                    end

                end
            end
        end
    end
end

assert(n_rows == n_ineq);

%% Objective
% Maximize position along track
if vh.approximationIsSL
    objective_lin(idx_pos(p.Hp, :)) = -p.Q * checkpoints(checkpoint_indices(p.Hp)).forward_vector;
elseif vh.approximationIsSCR
    objective_lin(idx_pos(p.Hp, :)) = -p.Q * track_polygons(track_polygon_indices(p.Hp)).forward_direction;
end

% Minimize control change over time (36)
objective_quad(idx_u(1,:), idx_u(1,:)) = p.R;
for i = 1:p.Hp - 1
    objective_quad(idx_u(  i, :), idx_u(  i, :)) = 2 * p.R;  
    objective_quad(idx_u(  i, :), idx_u(i+1, :)) = -p.R; 
    objective_quad(idx_u(i+1, :), idx_u(  i, :)) = -p.R;
end
objective_quad(idx_u(p.Hp, :), idx_u(p.Hp, :)) = p.R;

% Minimize slack var (parameter q in paper?)
objective_lin(idx_slack) = p.S;
%old: objective_quad(idx_slack, idx_slack) = 1e30;

%% Blocking: minimize lateral delta to opponent if blocking is recommended
if p.isBlockingEnabled
    n_affectedTrajSteps = 3;
    if ws.blockingTable(i_vehicle,1) == 1
        for k = 1:n_affectedTrajSteps
            % ease access
            checkpoint_c = checkpoints(checkpoint_indices(k));
            
            % nearest checkpoint of ego
            n_ego = checkpoint_c.normal_vector;
            T_ego = checkpoint_c.center;
            
            % nearest checkpoint of opp
            n0_opp = checkpoints(ws.vs{1,1}.cp_curr).normal_vector;
            T0_opp = checkpoints(ws.vs{1,1}.cp_curr).center;
            
            % current position of attacking opponent
            p0_opp = ws.vs{1,1}.x0(1:2);
            
            n1 = n_ego(1); n2 = n_ego(2);
            T1 = T_ego(1); T2 = T_ego(2);
            p1o = p0_opp(1); p2o = p0_opp(2);
            n1o = n0_opp(1); n2o = n0_opp(2);
            T1o = T0_opp(1); T2o = T0_opp(2);
            
            objective_quad(idx_pos(k,1),idx_pos(k,1)) = 64 * ( n1^4 + n1^2 * n2^2 );
            objective_quad(idx_pos(k,2),idx_pos(k,2)) = 64 * ( n1^2 * n2^2 + n2^4 );
            objective_quad(idx_pos(k,1),idx_pos(k,2)) = 64 * ( n1^3 * n2 + n1 * n2^3 );
            objective_quad(idx_pos(k,2),idx_pos(k,1)) = 64 * ( n1^3 * n2 + n1 * n2^3 );
            objective_lin(idx_pos(k,1)) = 8 * ( (-2)*T1*n1^4 + (-2)*T2*n1^3*n2 + (-2)*T1*n1^2*n2^2 + (-2)*T2*n1*n2^3 + (-2)*n1^3*p1o*n1o + (-2)*n1*n2^2*p1o*n1o + 2*n1^3*T1o*n1o + 2*n1*n2^2*T1o*n1o + (-2)*n1^3*p2o*n2o + (-2)*n1*n2^2*p2o*n2o + 2*n1^3*T2o*n2o + 2*n1*n2^2*T2o*n2o );
            objective_lin(idx_pos(k,2)) = 8 * ( (-2)*T1*n1^3*n2 + (-2)*T2*n1^2*n2^2 + (-2)*T1*n1*n2^3 + (-2)*T2*n2^4 + (-2)*n1^2*n2*p1o*n1o + (-2)*n2^3*p1o*n1o + 2*n1^2*n2*T1o*n1o + 2*n2^3*T1o*n1o + (-2)*n1^2*n2*p2o*n2o + (-2)*n2^3*p2o*n2o + 2*n1^2*n2*T2o*n2o + 2*n2^3*T2o*n2o );
        end
    end
end

%% Bounds
% Slack Var must be positive
bound_lower(idx_slack) = 0;

if isLinear
    % Bounded acceleration
    if vh.approximationIsSL
        bound_upper(idx_u(:)) =  p.a_max;
        bound_lower(idx_u(:)) = -p.a_max;

        % Trust region for change in position
        % Bounded states (trust region for change in position) - kinetic
        bound_upper(idx_pos(1:p.Hp, :)) = (x(model.ipos, 1:p.Hp) + p.trust_region)';
        bound_lower(idx_pos(1:p.Hp, :)) = (x(model.ipos, 1:p.Hp) - p.trust_region)';
    elseif vh.approximationIsSCR
        bound_upper(idx_u(:)) =  p.a_max;
        bound_lower(idx_u(:)) = -p.a_max;
    end
else
    % Bounded inputs
    % all time
    bound_upper(idx_u(:,1)) =  p.TR_steeringAngle;
    bound_upper(idx_u(:,2)) =  p.TR_motorTorque;
    bound_lower(idx_u(:,1)) = -p.TR_steeringAngle;
    bound_lower(idx_u(:,2)) = -p.TR_motorTorque;
    % last prediction step
    bound_upper(idx_u(end,1)) =  0.15 * p.TR_steeringAngle;
    bound_upper(idx_u(end,2)) =  0.15 * p.TR_motorTorque;
    bound_lower(idx_u(end,1)) = -0.15 * p.TR_steeringAngle;
    bound_lower(idx_u(end,2)) = -0.15 * p.TR_motorTorque;

    % Bounded states (trust region for change in position) - kinetic
    bound_upper(idx_pos(1:p.Hp,:)) = (x(model.ipos, 1:p.Hp) + p.TR_pos)';
    bound_lower(idx_pos(1:p.Hp,:)) = (x(model.ipos, 1:p.Hp) - p.TR_pos)';

    % Bounded states (trust region for change in velocities) - kinematic
    bound_upper(idx_x(:,3)) = p.TR_velX;
    bound_upper(idx_x(:,4)) = p.TR_velY;
    bound_upper(idx_x(:,6)) = p.TR_velW;
    bound_lower(idx_x(:,3)) = 0.005;
    bound_lower(idx_x(:,4)) = -p.TR_velY;
    bound_lower(idx_x(:,6)) = -p.TR_velW;

    % "Terminal Constraints"
    % Bounded states for last prediction step instead of term. constr.
    bound_upper(idx_x(end,3)) = 0.01;
    bound_upper(idx_x(end,4)) = 0.01;
    bound_upper(idx_x(end,6)) = 0.02;
    bound_lower(idx_x(end,3)) = 0.005;
    bound_lower(idx_x(end,4)) = -0.01;
    bound_lower(idx_x(end,6)) = -0.02;
end
end