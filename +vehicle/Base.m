classdef Base
    % wrapping all functions around a given model as an ODE & its
    % parameters:
    % linearization, discretization, ...
    
    properties
        p       % parameters of the vehicle
        
        nx      % Number of state variables
        nu      % Number of control/input variables
        ns      % Number of states + controls
        
        ix      % Indices of state variables
        ipos    % Indices of position variables - PART OF x
        iu      % Indices of control variables
    end
    
    methods
        function obj = Base(p, nx, nu)
            % TODO nx and nu have to match ODE
            obj.p = p;
            obj.nx = nx;
            obj.nu = nu;
            
            obj.ns = nx + nu;  
            obj.ix = 1:nx;
            obj.ipos = 1:2; 
            obj.iu = nx + (1:nu);
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