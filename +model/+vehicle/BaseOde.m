classdef BaseOde < model.vehicle.Base
    % base class for ODEs with symbolic differentiation and linearization
    
    properties
        ode_jacobian % symbolic Jacobian of the ODE
    end
    
    methods
        function obj = BaseOde(nx, nu, Hp, dt, p)
            % call superclass constructor
            obj@model.vehicle.Base(nx, nu, Hp, dt, p)
            
            % pre-calc symbolic linearization
            obj.ode_jacobian = obj.linearize_model();
        end
        
        function dx = calc_step(obj, x, u)
            dx = obj.ode(obj.p, x, u);
        end
        
        function [Ad, Bd, Ed] = calculatePredictionMatrices(obj, x0, u0)    
            % Initialize discretized matrices
            Ad = zeros(obj.nx, obj.nx, obj.Hp);
            Bd = zeros(obj.nx, obj.nu, obj.Hp);
            Ed = zeros(obj.nx, obj.Hp);

            % Calculate discretized prediction matrix for each prediction step
            for i = 1:obj.Hp
               [Ad(:,:,i), Bd(:,:,i), Ed(:,i)] =  ...
                   obj.discretize_linearized_model(x0(:,i), u0(:,i));
            end
        end
    end
        
    methods (Abstract)
        % defined by superclass, see there
        dX = ode(obj, x, u)
    end
    
    methods (Access = private)
        function linearized_model = linearize_model(obj)
            % create linearization function ODE of given model. Returns function
            % which linearizes model at given state and input

            %% Symbolic Calculations
            % create symbolic state and input variable
            % (name, [n by m array], variable is in set of real numbers)
            x0 = sym('x0_', [obj.nx, 1], 'real');
            u0 = sym('u0_', [obj.nu, 1], 'real');

            % calculate dx (ODE at 0)
            dx_0 = obj.ode(x0, u0); % TODO CAVE CommonRoad uses other arg order

            % compute Jacobian as symbolic expression
            Ac = jacobian(dx_0, x0); % derivation of dx to x
            Bc = jacobian(dx_0, u0); % derivation of dx to u
            Ec = dx_0 - Ac * x0 - Bc * u0; % Taylor-series? TODO

            %% convert all of above symbolic expression to function handle
            % with 3 outputs, using 'Vars' as inputs
            linearized_model = matlabFunction(Ac, Bc, Ec, 'Vars', {x0, u0});
        end
        
        function [Ad, Bd, Ed] = discretize_linearized_model(obj, x0, u0)
            % Adapt value of velocity in x-dircetion [ x0(3) = dx ] to avoid
            % NaN-results because system-ode devides by dx
            if x0(3) == 0
                x0(3) = eps; % assign smallest possible value in Matlab
            end

            % Get linearized model
            [Ac, Bc, Ec] = obj.ode_jacobian(x0, u0);

            %% Discretize model
            % (1)
            % Formula from wikipedia
            % https://en.wikipedia.org/wiki/Discretization#cite_ref-1
            % expm([A B; 0 0] * dt) == [Ad Bd; 0 eye]
            % Cited from
            % Raymond DeCarlo: Linear Systems: A State Variable Approach with Numerical Implementation, Prentice Hall, NJ, 1989
            % page 215
            tmp = expm(obj.dt*[Ac Bc; zeros(size(Ac,2)+size(Bc,2)-size(Ac,1),size(Ac,2)+size(Bc,2))]);
            Ad = tmp(1:size(Ac,1),1:size(Ac,2));
            Bd = tmp(1:size(Bc,1),[1:size(Bc,2)]+size(Ac,2));
            %     tmp2 = expm_new(dt*[Ac Bc; zeros(size(Ac,2)+size(Bc,2)-size(Ac,1),size(Ac,2)+size(Bc,2))]);
            %     Ad2 = tmp2(1:size(Ac,1),1:size(Ac,2));
            %     Bd2 = tmp2(1:size(Bc,1),[1:size(Bc,2)]+size(Ac,2));

            tmp = expm(obj.dt*[Ac Ec; zeros(size(Ac,2)+size(Ec,2)-size(Ac,1),size(Ac,2)+size(Ec,2))]);
            Ed = tmp(1:size(Ec,1), [1:size(Ec,2)]+size(Ac,2));

            % (2) Control System Toolbox (from Alrifaee 2017)
            %     Cc = [1 0 0 0 0 0;0 1 0 0 0 0];
            %     dt = model.p.dt;
            %     
            %     m = ss( Ac, Bc, Cc, zeros(size(Cc,1),size(Bc,2)) ); % Zero-Matrix corresponds to Dc
            %     md = c2d( m, dt );
            %     Ad = md.a;
            %     Bd = md.b;
            % 
            %     maff  = ss( Ac,Ec,Cc,zeros(size(Cc,1),1) );
            %     mdaff = c2d(maff,dt);
            %     Ed = mdaff.b;

            % (3) Symbolic Toolbox
            %     dt = model.p.dt;
            %     M = dt*[Ac Bc; zeros(size(Ac,2)+size(Bc,2)-size(Ac,1),size(Ac,2)+size(Bc,2))];
            %     tmp = expm(M);
        end
    end
end