classdef Base < handle
    % wrapping all functions around a given model as an ODE & its
    % parameters:
    % linearization, discretization, ...
    
    properties
        p       % parameters of the vehicle
        Hp      % prediction horizon
        dt      % time-step size
        
        nx      % Number of state variables
        nu      % Number of control/input variables
        ns      % Number of states + controls
        
        idx_x   % Indices of state variables
        idx_pos % Indices of position variables - PART OF x
        idx_u   % Indices of control variables
    end
    
    methods
        function obj = Base(nx, nu, Hp, dt, p)
            % NOTE nx and nu have to match ODE
            obj.p = p;
            obj.nx = nx;
            obj.nu = nu;
            
            obj.Hp = Hp;
            obj.dt = dt;
            
            obj.ns = nx + nu;  
            obj.idx_x = 1:nx;
            obj.idx_pos = 1:2; 
            obj.idx_u = nx + (1:nu);
        end
        
        function dx = calc_step(obj, x, u)
            dx = obj.ode(obj.p, x, u);
        end
    end
        
    methods (Abstract)
        % Calculate discretized and linearized state-space matrices at
        % given working points (being state and input; given for
        % every predcition step) at every given step
        [Ad, Bd, Ed] = calculatePredictionMatrices(obj, x0, u0)
        
        % Model-defining ODE (ordinary differential equation)
        dX = ode(obj, x, u)
    end
end