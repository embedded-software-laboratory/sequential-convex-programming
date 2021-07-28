classdef SingleTrack < model.vehicle.BaseOde
    % input: /delta steering angle, /tau torque rear wheels
    % as defined in A. Liniger, ‘Path Planning and Control for Autonomous Racing’, Dissertation, ETH Zurich, 2018.
    methods (Static)
        function p = getParamsLinigerRC_1_43_WithLinigerBounds
            % Liniger RC 1:43 MPCC from GitHub https://github.com/alexliniger/MPCC/blob/84cc61d628a165a424c805bbe071fe96b88da2d0/Matlab/getModelParams.m
            % Bounds adapted from https://github.com/alexliniger/MPCC/blob/84cc61d628a165a424c805bbe071fe96b88da2d0/Matlab/getMPC_vars.m
            
            p.paramsName = 'ST from Liniger 1:43';
            
            % Tire
            p.Br = 3.3852;
            p.Cr = 1.2691;
            p.Dr = 0.1737;

            p.Bf = 2.579;
            p.Cf = 1.2;
            p.Df = 0.192;
            
            % Vehicle
            p.m = 0.041; % vehicle mass [kg]
            p.Iz = 27.8e-6; % vehicle inertia [kg m^2]
            p.l_f = 0.029; % front wheel to CoG [m]
            p.l_r = 0.033; % rear wheel to CoG [m]
            
            % Acceleration/Drag
            p.Cm1 = 0.287;
            p.Cm2 = 0.0545;
            p.Cr0 = 0.0518;
            p.Cr2 = 0.00035;
            
            % Bounds (first row lower, second upper bounds)
            %    states                          inputs
            %    p_x  p_y  v_x  v_y  yaw  dyaw   delta d
            %    m    m    m/s  m/s       1/s    rad   %
            % CAVE position bounds are not considered in QP creation
            p.bounds = ...
                [-Inf -Inf -.1  -2   -10  -7    -.35  -.1 ;
                  Inf  Inf  4    2    10   7     .35   1 ];
            % CAVE delta not considered in QP creation
            %p.bounds_delta = ...
            %    [-Inf -Inf -Inf -Inf -Inf -Inf   -1   -1  ;
            %      Inf  Inf  Inf  Inf  Inf  Inf    1    1 ];
        end
        
        function p = getParamsKloockRC_1_43_WithLinigerBounds
            % Liniger, acc. to M. Kloock, P. Scheffe, L. Botz, J. Maczijewski, B. Alrifaee, and S. Kowalewski, ‘Networked Model Predictive Vehicle Race Control’, in 2019 IEEE Intelligent Transportation Systems Conference (ITSC), Auckland, New Zealand, Oct. 2019, pp. 1552–1557, doi: 10.1109/ITSC.2019.8917222.
            
            % get defaults, then overwrite changes
            p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
            
            p.paramsName = 'ST from Kloock via Plots ST Liniger 1:43';
            
            % Tire (CAVE: being different than Liniger GitHub's)
            p.Bf = 3.0323;
            p.Cf = 1.9669;
            p.Df = 0.1884;
            
            p.Br = 4.1165;
            p.Cr = 1.3709;
            p.Dr = 0.1644;
            
            % Acceleration/Drag
            % Kloock simplified formula to "n*T",
            %   n beeing transmission ratio
            %   T being the input torque
            % this model can be represented with the following parameters
            p.Cm1 = 1;
            p.Cm2 = 0;
            p.Cr0 = 0;
            p.Cr2 = 0;
        end
        
        function p = getParamsKloockRC_1_43_WithKloocksBounds
            % get defaults, then overwrite changes
            p = model.vehicle.SingleTrack.getParamsKloockRC_1_43_WithLinigerBounds();
            
            p.paramsName = 'ST from Kloock via Plots ST Liniger 1:43, plus custom bounds';
            
            % Bounds (first row upper, second lower bounds)
            %    states                          inputs
            %    p_x  p_y  v_x  v_y  yaw  dyaw   delta d
            %    m    m    m/s  m/s       1/s    rad   [CAVE] Nm
            % CAVE position bounds are not considered in QP creation
            p.bounds = ...
                [-Inf -Inf  .05 -2   -Inf -2*pi  -.4   -.08 ;
                  Inf  Inf  2    2    Inf  2*pi   .4    .08];
            % CAVE delta not considered in QP creation
            %p.bounds_delta = ...
            %    [-Inf -Inf -Inf -Inf -Inf -Inf   -Inf  -Inf ;
            %      Inf  Inf  Inf  Inf  Inf  Inf    Inf   Inf];
        end
    end
    
    methods (Access = private)
        function f = accelerationEquations(obj, delta, torque, dyaw, v_x, v_y, u_1_lin)
            % Equations copied from method "ode"
            % all variables in local vehicle reference frame
            
            % current desired local acceleration values
            a_x = u_1_lin(1);
            a_y = u_1_lin(2);

            %% Tire forces
            % front/rear tire side slip angle
            alpha_f = -atan2(dyaw * obj.p.l_f + v_y, v_x) + delta;
            alpha_r =  atan2(dyaw * obj.p.l_r - v_y, v_x);
            % front/rear tire lateral force
            F_fy = obj.p.Df * sin(obj.p.Cf * atan(obj.p.Bf * alpha_f));
            F_ry = obj.p.Dr * sin(obj.p.Cr * atan(obj.p.Br * alpha_r));
            % rear tire longitudinal force
            F_rx = (obj.p.Cm1 - obj.p.Cm2 * v_x) * torque - obj.p.Cr0 - obj.p.Cr2 * v_x^2;

            %%
            % rearranged to fit form 0 = F(x)
            f(1) = -a_x + 1/obj.p.m * (F_rx - F_fy * sin(delta) + obj.p.m * v_y * dyaw);
            f(2) = -a_y + 1/obj.p.m * (F_ry + F_fy * cos(delta) - obj.p.m * v_x * dyaw);
        end
    end
    
    properties
        fsolve_options
    end

    methods
        function obj = SingleTrack(Hp, dt, p)
            obj@model.vehicle.BaseOde(6, 2, Hp, dt, p) % call superclass constructor
            
            warning('Acceleration controller for linear model is experimental, use with caution')
            
            obj.fsolve_options = optimoptions('fsolve','Display','off');
        end
        
        function dX = ode(obj, x, u)
            % The kinetic bicycle model from Liniger et al. (2014) is used and
            % slightly adapted to represent the vehicle dynamics. The model used in
            % Liniger et al. (2014) is based on Velenis et al. (2009) and Voser et
            % al. (2010), tire forces are modeled based on a simplified Pacejka
            % Tire Model presented in Bakker et al. (1987). The longitudinal force
            % of the rear wheel (Fr_x) is simplified.

            % Inputs:   p (parameter struct), X (vector of current system states),
            %           u (vector of current system inputs)
            % Ouputs:   dX (vector of first order differential equations)
            
            % States:
            % 1 x-position of vehicle CoG in intertial frame
            % 2 y-position of vehicle CoG in intertial frame
            % 3 longitudinal velocity of vehicle CoG
            % 4 lateral velocity of vehicle CoG
            % 5 vehicle angle relative to inertial frame (yaw)
            % 6 yaw rate
            
            % Inputs
            % 1 steering angle
            % 2 motor torque

            %% Readability
            % States
            v_x   = x(3);
            v_y   = x(4);
            yaw   = x(5);
            dyaw  = x(6);
            % Inputs
            delta = u(1);
            t     = u(2);

            %% Tire forces
            % front/rear tire side slip angle
            alpha_f = -atan2(dyaw * obj.p.l_f + v_y, v_x) + delta;
            alpha_r =  atan2(dyaw * obj.p.l_r - v_y, v_x);
            % front/rear tire lateral force
            F_fy = obj.p.Df * sin(obj.p.Cf * atan(obj.p.Bf * alpha_f));
            F_ry = obj.p.Dr * sin(obj.p.Cr * atan(obj.p.Br * alpha_r));
            % rear tire longitudinal force
            F_rx = (obj.p.Cm1 - obj.p.Cm2 * v_x) * t - obj.p.Cr0 - obj.p.Cr2 * v_x^2;

            %% ODE
            dX = [
                v_x * cos(yaw) - v_y * sin(yaw);
                v_x * sin(yaw) + v_y * cos(yaw);
                1/obj.p.m * (F_rx - F_fy * sin(delta) + obj.p.m * v_y * dyaw);
                1/obj.p.m * (F_ry + F_fy * cos(delta) - obj.p.m * v_x * dyaw);
                dyaw;
                1/obj.p.Iz * (F_fy * obj.p.l_f * cos(delta) - F_ry * obj.p.l_r)];
        end
        
        function u_1_ST = acceleration_controller(obj, x_0, u_1_lin, u_1_ST_prev)
            % CAVE EXPERIMENTAL
            % acceleration controller: converting from desired acceleration
            % to single-track inputs (namely torque and steering angle)
            %
            % Inputs
            %   u_1_lin is controller output (global acceleration values)
            %   u_1_ST_prev is previous ST output (for this specific
            %       vehicle with given x_0 and u_1_lin

            %% Parameters
            v_long = x_0(3);
            v_lat  = x_0(4);
            yaw    = x_0(5);
            dyaw   = x_0(6);
            
            % convert global to vehicle reference frame
            a_local = model.vehicle.vector_global2local(u_1_lin, yaw);
            
            % prevent division by zero in ST model
            if v_long == 0
               v_long = 5* eps; 
            end

            % create wrapper for fsolve
            f = @(x) obj.accelerationEquations(x(1), x(2), dyaw, v_long, v_lat, a_local);
            
            [u_1_ST, ~, exitflag] = fsolve(f, u_1_ST_prev, obj.fsolve_options);
            % TODO possibly use `UseParallel`?
           
            % FIXME check input limits (even better: at some central instance)
            
            if exitflag <= 0
                % TODO better avoidance strategy?
                % FIXME stable avoidance strategy
                warning('acceleration controller solver failed with exitflag %i, setting u_1_ST conservatively to zeros', exitflag);
                %u_1_ST = 0 .* u_1_ST;
                %u_1_ST = u_1_ST_prev;
                k = 0.6; u_1_ST = (1-k) .* 0.5 .* u_1_ST + k .* u_1_ST_prev;
            end
            
            fprintf('accel_ctrl: (with v_long %.2f, v_lat %.2f, yaw %.2f, dyaw %.2f)\n\t(a_x, a_y) (%.2f, %.2f) -> (a_long, a_lat) (%.2f, %.2f)-> delta %.2f, torque %.2f\n',...
                v_long, v_lat, yaw, dyaw,...
                u_1_lin(1), u_1_lin(2), a_local(1), a_local(2), u_1_ST(1), u_1_ST(2));
         end
    end
end