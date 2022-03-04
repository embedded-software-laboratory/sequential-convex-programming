function [n_vars, idx_x, idx_u, idx_slack, objective_quad, objective_lin,...
    A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper] = ...
        create_QP(...
            vhs_cfg, x_0, X_opt_prev, U_opt_prev,...
            checkpoints, checkpoint_indices,...
            track_polygons, track_polygon_indices,...
            iter, i_vehicle,...
            obstacleTable, blockingTable, vhs_ws)
% QP formulation: create and solve convexified problem
% track can be represented as SCR or SL
% SL: Sequential Linearization controller: QP approximated (linearised, thus relaxed) variant
% SCR: Sequential Convex Restriction controller: QP in a restrictive variant
% 
% Inputs
%   X_opt: last iterations calculated optimal states
%   U_opt: last iterations calculated optimal inputs
%   vhs: current vehicle working set
%       requires fields x_0, X_opt, cp_curr

% Assign for better readability/access
vh_cfg = vhs_cfg{i_vehicle};
vhP = vh_cfg.p;
vhModel = vh_cfg.model_controller;
isControlModelLinear = vh_cfg.isControlModelLinear;

if vh_cfg.approximationIsSL
    % Verification if indices-matrix has only one row and Hp columns
    assert(size(checkpoint_indices, 1) == 1);
    assert(size(checkpoint_indices, 2) == vhP.Hp);
elseif vh_cfg.approximationIsSCR
    % Verification if indices-matrix has only one column and Hp rows
    assert(size(track_polygon_indices, 1) == vhP.Hp);
    assert(size(track_polygon_indices, 2) == 1);
end

%% Index mapping
idx        = reshape((1:vhModel.ns * vhP.Hp), vhModel.ns, vhP.Hp)';
idx_x      = idx(:, vhModel.idx_x);
idx_pos    = idx(:, vhModel.idx_pos);
idx_u      = idx(:, vhModel.idx_u);
idx_slack  = vhModel.ns * vhP.Hp + 1;

%% Problem size
n_vars = vhModel.ns * vhP.Hp + 1; % number of variables: (states + inputs) * prediction steps + slack variable
n_eqns = vhModel.nx * vhP.Hp; % number of equations: state equations * prediction steps
if isControlModelLinear
    n_eqns = n_eqns + 2; % terminating conditions (v_x, v_y)
else
    % terminal constraints are implemented as (softer) lower/upper bounds
    % on prediction step H_p (see below, section "bounds")
    %n_eqns = n_eqns + 3; % terminating conditions (v_x, v_y, dyaw/dt)
end
n_ineq = 0;


if vh_cfg.approximationIsSL
    % 2 track constraints at every time step 
    n_ineq = n_ineq + 2 * vhP.Hp;
elseif vh_cfg.approximationIsSCR
    % n-polygon-dependent track constraints at every time step
    for k = 1:vhP.Hp
        n_ineq = n_ineq + length(track_polygons(track_polygon_indices(k)).b);
    end
end

if isControlModelLinear
    n_ineq = n_ineq + vhP.n_acceleration_limits * vhP.Hp; % acceleration bounds at every time step
end
if vhP.areObstaclesConsidered
    % at first iteration no obstacle constraints are respected
    if ~((sum(obstacleTable(i_vehicle,:)) == 0) || ((sum(obstacleTable(i_vehicle,:)) ~= 0) && (iter == 1)))
        n_ineq = n_ineq + sum(obstacleTable(i_vehicle,:)) * vhP.Hp;  % relevant obstacle constraints at every time step
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
%   x_{k+1] = Ad * x_k + Bd * u_{k+1} + Ed
%       -> linearization requires combination of x_k and u_{k+1}
%   NOTE linear models give back static prediction matrices
%       (-> independent of previous optimal trajectory)
%   x_0 x_1 x_2 ... x_{n-1} 
%       x are already shifted, was
%       x_{1,controller} -> x_{0,sim}         -> x_0
%       x_{2,controller} -> x_{2,controller}  -> x_1
%       .
%       .
%       .
%
%   u_1 u_2 u_3 ... u_n
[Ad, Bd, Ed] = vhModel.calculatePredictionMatrices(...
    [x_0 X_opt_prev(:, 1:end-1)],...
    U_opt_prev);

% k = 1
n_rows = n_rows(end) + (1:vhModel.nx);
A_eq(n_rows, idx_x(1,:)) = eye(vhModel.nx);    % Coeff. x_k+1
A_eq(n_rows, idx_u(1,:)) = -Bd(:,:,1);       % Coeff. u_k+1
b_eq(n_rows) = Ad(:,:,1) * x_0 + Ed(:,1);    % Ad * x_k + Ed with x_k = x_0 (current state)

% k = 2:Hp
for k = 2:vhP.Hp
    n_rows = n_rows(end) + (1:vhModel.nx);
    A_eq(n_rows, idx_x(k,:)) = eye(vhModel.nx); % Coeff. x_k+1
    A_eq(n_rows, idx_x(k-1,:)) = -Ad(:,:,k);  % Coeff. x_k
    A_eq(n_rows, idx_u(k,:)) = -Bd(:,:,k);    % Coeff. u_k+1
    b_eq(n_rows) = Ed(:,k);                   % Ed
end

%% Terminating conditions
if isControlModelLinear
    % v_{x,y}{Hp} = 0 (b_eq already inited to 0)
    n_rows = n_rows(end) + (1:2);
    A_eq(n_rows, idx_x(k, 3:4)) = eye(2); %v_x, v_y
else
    % terminal constraints are implemented as (softer) lower/upper bounds
    % on prediction step H_p (see below, section "bounds")
    %n_rows = n_rows(end) + 1;
    %A_eq(n_rows, idx_x(k, 3)) = 1; % v_x
    %n_rows = n_rows(end) + 1;
    %A_eq(n_rows, idx_x(k, 4)) = 1; % v_y
    %n_rows = n_rows(end) + 1;
    %A_eq(n_rows, idx_x(k, 6)) = 1; % dyaw/dt
end

assert(n_rows(end) == n_eqns);

%% Inequalities

% Override obstacle constraint (set to zero) if trajectory point is ahead of corresponding obstacle trajectory point
% TODO shouldn't be this reset every iteration? How are multiple obstacles handled?
isOpponentOvertaken = false;

n_rows = 0; % for double check against defined problem size above
for k = 1:vhP.Hp
    %% Track limits
    % assign current checkpoint for better readability
    checkpoint_c = checkpoints(checkpoint_indices(k));
    
    if vh_cfg.approximationIsSL
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
    elseif vh_cfg.approximationIsSCR
        track_polygon = track_polygons(track_polygon_indices(k));
        n_rows = n_rows(end) + (1:length(track_polygon.b));
        A_ineq(n_rows, idx_slack) = -1;
        A_ineq(n_rows, idx_pos(k,:)) = track_polygon.A;
    
        b_ineq(n_rows) = track_polygon.b;
        n_rows = n_rows(end);
    end
    
    if isControlModelLinear
        %% Acceleration limits
        % inequalities required due to complex shape of input limits due to
        % not modelling the actual technical inputs but the resulting
        % accelerations
        [Au_acc, b_acc] = controller.get_acceleration_ellipses(...
            vh_cfg.modelParams_controller, vhP, (1:vhP.n_acceleration_limits)', X_opt_prev(:, k));
        n_rows = n_rows(end) + vhP.n_acceleration_limits;
        
        A_ineq(n_rows - vhP.n_acceleration_limits + 1:n_rows, idx_u(k,:)) = Au_acc;
        b_ineq(n_rows - vhP.n_acceleration_limits + 1:n_rows) = b_acc;
    end


    %% Obstacle constraints and override/contraction of track constraints (only in 2nd iteration, only if obstacles are to be respected)
    % TODO: why only in second iteration
    % TODO: add warning when obstacles enabled, but p.iteration == 1
    if vhP.areObstaclesConsidered
        
        % TODO: see below outcommented, too
        %         % Override (constrict) track constraint on one side if opponent is significantly alongside and ego has more space (on the inside)
        %         trackCenter2ego_vec = (x(1:2,k) - checkpoint_c.center)' * left_unit_vector;
        %         trackCenter2obst_vec = (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector;
        %         xOpp = trajectories.vehicles{1,2}.X_opt(:,k);
        %         if ( norm(trackCenter2ego_vec,2) < norm(trackCenter2obst_vec,2) ) && ...
        %             ( blockingTable(vehNr,1) == 1 ) && ...
        %             ( abs( (x(1:2,k) - xOpp(1:2))' *  checkpoint_c.forward_vector ) <= 0.075 ) && ...
        %             ( j ~= vehNr )
        %             if (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector > 0 % obstacle on left side
        %                 b_ineq(n_rows-1) = left_unit_vector' * (checkpoint_c.left - left_unit_vector * 0.045);
        %             elseif (xOpp(1:2) - checkpoint_c.center)' * left_unit_vector < 0 % obstacle on right side
        %                 b_ineq(n_rows) = -left_unit_vector' * (checkpoint_c.right + left_unit_vector * 0.045);
        %             end
        %         end

        if (iter >= 2) && (sum(obstacleTable(i_vehicle,:)) >= 1)

            % iterate over all opponents
            for j = 1:length(vhs_ws)
                % Extend predicted trajectory of opponent if the opponent's prediction horizon is shorter than the own horizon
                % TODO: very simplistic approach
                if k <= size(vhs_ws{j}.X_opt,2) % Choose correponding trajectory point if prediciton step is within scope
                    X_opt_opp = vhs_ws{j}.X_opt(:, k);
                else % If prediction step is not within scope, choose last trajectory point
                    X_opt_opp = vhs_ws{j}.X_opt(:, end);
                end    

                % Respect only constraints for opponents marked as obstacles
                if obstacleTable(i_vehicle,j) == 1
                    d = pdist([X_opt_prev(1:2,k)';X_opt_opp(1:2)'],'euclidean'); % calculate distance
                    normal_vector = - (X_opt_prev(1:2,k)-X_opt_opp(1:2))/d; % normal vector in direction from trajectory point to obstacle center
                    V_diff = X_opt_prev(3:4,k) - X_opt_opp(3:4);    % velocity difference between ego and opponent

                    if strcmp(vhs_cfg{j}.distSafe, 'Circle')
                        D_vel_1 = sqrt((V_diff(1) * vhP.dt_controller)^2 + (V_diff(2) * vhP.dt_controller)^2);  % safety distance due to velocity of objects
                        D_size_1 = 2*vhs_cfg{j}.distSafe2CenterVal_1;       % safety distance due to object size
                        Dsafe_1 = D_vel_1 + D_size_1;
                        closest_obst_point = X_opt_opp(1:2) - normal_vector * Dsafe_1; % intersection of safe radius and connection between trajectory point and obstacle center
                    elseif strcmp(vhs_cfg{j}.distSafe, 'Ellipse')
                        D_vel_2 = ( normal_vector' * V_diff ) * vhP.dt_controller;
                        yaw_ang = X_opt_prev(5,k);
                        Rot_yaw = [ cos(yaw_ang) -sin(yaw_ang) ; sin(yaw_ang) cos(yaw_ang) ];
                        D_size_2 = normal_vector' * (Rot_yaw * vhs_cfg{j}.distSafe2CenterVal_2);
                        Dsafe_2 = norm( D_vel_2 + D_size_2 );
                        closest_obst_point = X_opt_opp(1:2) - normal_vector * Dsafe_2; % intersection of safe radius and connection between trajectory point and obstacle center
                    elseif strcmp(vhs_cfg{j}.distSafe, 'CircleImpr')
                        D_vel_1 = ( normal_vector' * V_diff ) * vhP.dt_controller; % Improved
    %                         D_vel_1 = 0;
                        D_size_1 = 2*vhs_cfg{j}.distSafe2CenterVal_1;
                        Dsafe_1 = D_vel_1 + D_size_1;
                        closest_obst_point = X_opt_opp(1:2) - normal_vector * Dsafe_1;
                    elseif strcmp(vhs_cfg{j}.distSafe, 'EllipseImpr')
                        D_vel_2 = ( normal_vector' * V_diff ) * vhP.dt_controller;
                        yaw_ang = X_opt_opp(5); % FIXME add support for linear model: calculate yaw angle
                        Rot_yaw = [ cos(yaw_ang) -sin(yaw_ang) ; sin(yaw_ang) cos(yaw_ang) ];
    %                         D_size_2 = norm( (-normal_vector) .* (Rot_yaw * vhs{1,j}.distSafe2CenterVal_2) );
    %                         Dsafe_2 = D_vel_2 + D_size_2;
    %                         closest_obst_point = xOpp(1:2) - normal_vector * Dsafe_2;
                        safety_ellipseA = Rot_yaw * vhs_cfg{j}.distSafe2CenterVal_2;
                        closest_obst_point = X_opt_opp(1:2) + [ safety_ellipseA(1)*(-normal_vector(1)) ; safety_ellipseA(2)*(-normal_vector(2)) ]  - normal_vector * D_vel_2;
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
if vh_cfg.approximationIsSL
    objective_lin(idx_pos(vhP.Hp, :)) = -vhP.Q * checkpoints(checkpoint_indices(vhP.Hp)).forward_vector;
elseif vh_cfg.approximationIsSCR
    objective_lin(idx_pos(vhP.Hp, :)) = -vhP.Q * track_polygons(track_polygon_indices(vhP.Hp)).forward_direction;
end

% Minimize control change over time
% create repetetive structure
for i = 1:vhP.Hp - 1
    objective_quad(idx_u(  i, :), idx_u(  i, :)) = 2 * vhP.R;  
    objective_quad(idx_u(  i, :), idx_u(i+1, :)) = -vhP.R; 
    objective_quad(idx_u(i+1, :), idx_u(  i, :)) = -vhP.R;
end
% overwrite repetetive for first & last entry
objective_quad(idx_u(1,:), idx_u(1,:)) = vhP.R;
objective_quad(idx_u(vhP.Hp, :), idx_u(vhP.Hp, :)) = vhP.R;

% Minimize slack var
objective_lin(idx_slack) = vhP.S;
% alternatively add quadratic slack objective, too
% `objective_quad(idx_slack, idx_slack)` = vhP.S;

%% Blocking: minimize lateral delta to opponent if blocking is recommended
if vhP.isBlockingEnabled
    n_affectedTrajSteps = 3;
    if blockingTable(i_vehicle,1) == 1
        for k = 1:n_affectedTrajSteps
            % ease access
            checkpoint_c = checkpoints(checkpoint_indices(k));
            
            % nearest checkpoint of ego
            n_ego = checkpoint_c.normal_vector;
            T_ego = checkpoint_c.center;
            
            % nearest checkpoint of opp
            n0_opp = checkpoints(vhs_ws{1,1}.cp_curr).normal_vector;
            T0_opp = checkpoints(vhs_ws{1,1}.cp_curr).center;
            
            % current position of attacking opponent
            p0_opp = vhs_ws{1,1}.x_0(1:2);
            
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

% Trust Region for change in position
% required due to linearization at working point - if too far away,
% linearization error could increase too much
% (could use delta bounds instead, but not supported by solvers)
if vh_cfg.approximationIsSL
    bound_lower(idx_pos) = (X_opt_prev(vhModel.idx_pos, :) - vhP.trust_region_size)';
    bound_upper(idx_pos) = (X_opt_prev(vhModel.idx_pos, :) + vhP.trust_region_size)';
end

% Additional State and Input bounds
% only sensible for ODE models, as linear models'...
%   - inputs are restricted via inequalities
%   - states are only position and acceleration, of which...
%       - position is only restircted via trust region (above)
%       - acceleration is already restricted via inequalities (ellipses)
if ~isControlModelLinear
    % Bounded inputs
    bound_lower(idx_u) = repmat(vhModel.p.bounds(1, vhModel.idx_u), length(idx_u), 1);
    bound_upper(idx_u) = repmat(vhModel.p.bounds(2, vhModel.idx_u), length(idx_u), 1);
    % special for last prediction step
    %bound_lower(idx_u(end, :)) = 0.15 * bound_lower(idx_u(end, :));
    %bound_upper(idx_u(end, :)) = 0.15 * bound_upper(idx_u(end, :));

    % Bounded states
    % (all except position, as this is done above via trust region if required)
    bound_lower(idx_x(:, 3:end)) = repmat(vhModel.p.bounds(1, vhModel.idx_x(3:end)), length(idx_x(:, 3:end)), 1);
    bound_upper(idx_x(:, 3:end)) = repmat(vhModel.p.bounds(2, vhModel.idx_x(3:end)), length(idx_x(:, 3:end)), 1);

    % Terminal Constraints
    % 	Choosing small numerical values for last prediction step
	% 	(instead of terminal equality)
    %
    %   Overwriting bounds from above
    bound_upper(idx_x(end,3)) = 0.01; % v_long
    bound_upper(idx_x(end,4)) = 0.01; % v_lat
    bound_upper(idx_x(end,6)) = 0.02; % dyaw
    bound_lower(idx_x(end,3)) = 0.005;
    bound_lower(idx_x(end,4)) = -0.01;
    bound_lower(idx_x(end,6)) = -0.02;
end
end