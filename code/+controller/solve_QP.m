function [X_opt, U_opt, log] =...
    solve_QP(...
        isCplexAvailable, n_vars, idx_x, idx_u, idx_slack,...
        quad_objective, lin_objective,...
        A_ineq, b_ineq, A_eq, b_eq, bound_lower, bound_upper)
%% Solve QP
log = struct;

% use CPLEX to solve QP, if available, else fallback to MATLAB
if isCplexAvailable
    [qp_vars,log.fval,log.exitflag,log.output,log.lambda] = ...
        cplexqp(quad_objective,lin_objective,A_ineq,b_ineq,A_eq,b_eq,bound_lower,bound_upper,[],[]);
    
    % Exitflag values: (acc. to
    % https://www.ibm.com/docs/en/icos/12.9.0?topic=functions-cplexqp,
    % refering to https://www.ibm.com/docs/en/icos/12.9.0?topic=functions-cplexmiqcp)
    
    % "If you want to write the model out to a file, then you need to set:
    % opt.exportmodel = 'myModel.lp';"
    switch log.exitflag
        % case ~=1 and >=0: CPLEX qp has problematic solution
        case 6; warning('CPLEX qp exitflag 6: Non-optimal Solution available.');
        case 5; warning('CPLEX qp exitflag 5: Solution with numerical issues.');
        case 1 %disp('CPLEX qp exitflag 1: Function converged to a solution x.');
        case 0; warning('CPLEX qp exitflag 0: Number of iterations exceeded options.MaxIter.');
        % case < 0: CPLEX qp has no solution
        case -1; warning('CPLEX qp exitflag -1: Aborted.');
        case -2; warning('CPLEX qp exitflag -2: No feasible point was found.');
        case -3; warning('CPLEX qp exitflag -3: Problem is unbounded.');
        case -4; warning('CPLEX qp exitflag -4: NaN value was encountered during execution of the algorithm.');
        case -5; warning('CPLEX qp exitflag -5: Both primal and dual problems are infeasible.');
        case -7; warning('CPLEX qp exitflag -7: Search direction became too small. No further progress could be made.');
        case -8; warning('CPLEX qp exitflag -8: Problem is infeasible or unbounded.');
        case -9; warning('CPLEX qp exitflag -9: Limit reached.');
        otherwise; warning('CPLEX qp exitflag: unknown');
    end
else
    options = optimoptions('quadprog', 'Display', 'none');
    quad_objective_sp = sparse(quad_objective);
    [qp_vars,log.fval,log.exitflag,log.output,log.lambda] = ...
        quadprog(quad_objective_sp,lin_objective,A_ineq,b_ineq,A_eq,b_eq,bound_lower,bound_upper,[],options); 
    
    % exitflags acc. to quadprog documentation
    switch log.exitflag
        % case ~=1 and >=0: MATLAB quadprog has problematic solution
        case 1 %disp('MATLAB quadprog exitflag 1: Function converged to the solution x.');
        case 0; warning('MATLAB quadprog exitflag 0: Number of iterations exceeded options.MaxIterations.');
        % case < 0: MATLAB quadprog has no solution
        case -2; warning('MATLAB quadprog exitflag -2: Problem is infeasible. Or, for "interior-point-convex", the step size was smaller than options.StepTolerance, but constraints were not satisfied.');
        case -3; warning('MATLAB quadprog exitflag -3: Problem is unbounded.');
        % "'interior-point-convex' Algorithm"-specific exit flags (default algorithm)
        case 2; warning('MATLAB quadprog exitflag 2: Step size was smaller than options.StepTolerance, constraints were satisfied.');
        case -6; warning('MATLAB quadprog exitflag -6: Nonconvex problem detected.');
        case -8; warning('MATLAB quadprog exitflag -8: Unable to compute a step direction.');           
        otherwise; warning('MATLAB quadprog exitflag: unknown');
    end
end

if (length(qp_vars) == n_vars) && isnan(qp_vars(1))
    error('Solver exceeds time limit!\n');
elseif length(qp_vars) ~= n_vars
%     options = cplexoptimset('ExportModel', 'failedProblem.lp');
%     [qp_vars,optimization_log.fval,optimization_log.exitflag,optimization_log.output,optimization_log.lambda] = ...
%         quadprog(quad_objective_sp,lin_objective,A_ineq,b_ineq,A_eq,b_eq,bound_lower,bound_upper,[],options);
    error('Solver failed!');
end

% save results
X_opt = qp_vars(idx_x)';
U_opt = qp_vars(idx_u)';
log.slack = qp_vars(idx_slack);

if log.slack > 1e-10
    warning('Slackened track constraints with slack = %.2e!', log.slack)
end
end