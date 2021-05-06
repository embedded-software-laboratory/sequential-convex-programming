classdef SingleTrack < model.vehicle.BaseOde
    % input: /delta steering angle, /tau torque rear wheels
    % as defined in A. Liniger, ‘Path Planning and Control for Autonomous Racing’, Dissertation, ETH Zurich, 2018.
    methods (Static)
        function p = getParamsLinigerRC_1_43
            % Liniger RC 1:43 MPCC from GitHub https://github.com/alexliniger/MPCC/blob/84cc61d628a165a424c805bbe071fe96b88da2d0/Matlab/getModelParams.m
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
        end
        
        function p = getParamsKloockRC_1_43
            % Liniger, acc. to M. Kloock, P. Scheffe, L. Botz, J. Maczijewski, B. Alrifaee, and S. Kowalewski, ‘Networked Model Predictive Vehicle Race Control’, in 2019 IEEE Intelligent Transportation Systems Conference (ITSC), Auckland, New Zealand, Oct. 2019, pp. 1552–1557, doi: 10.1109/ITSC.2019.8917222.
            
            % Tire (CAVE: being different than Liniger GitHub's)
            p.Bf = 3.0323;
            p.Cf = 1.9669;
            p.Df = 0.1884;
            
            p.Br = 4.1165;
            p.Cr = 1.3709;
            p.Dr = 0.1644;
            
            % Vehicle
            p.m = 0.041; % vehicle mass [kg]
            p.Iz = 27.8e-6; % vehicle inertia [kg m^2]
            p.l_f = 0.029; % front wheel to CoG [m]
            p.l_r = 0.033; % rear wheel to CoG [m]
            
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
    end

    methods
        function obj = SingleTrack(Hp, dt, p)
            % inputs
            p.idx_d = 1; % steering angle
            p.idx_t = 2; % motor torque
            
            % states
            p.idx_pos_x = 1;    % x-position of vehicle CoG in intertial frame
            p.idx_pos_y = 2;    % y-position of vehicle CoG in intertial frame
            p.idx_v_x = 3;  % longitudinal velocity of vehicle CoG
            p.idx_v_y = 4;  % lateral velocity of vehicle CoG
            p.idx_yaw = 5;	% vehicle angle relative to inertial frame (yaw)
            p.idx_dyaw = 6;	% yaw rate
            
            obj@model.vehicle.BaseOde(6, 2, Hp, dt, p) % call superclass constructor
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

            %% Readability
            % States
            v_x   = x(obj.p.idx_v_x);
            v_y   = x(obj.p.idx_v_y);
            yaw   = x(obj.p.idx_yaw);
            dyaw  = x(obj.p.idx_dyaw);
            % Inputs
            delta = u(obj.p.idx_d);
            t     = u(obj.p.idx_t);

            %% Tire forces
            % front/rear tire side slip angle
            alpha_f = -atan2(dyaw * obj.p.l_f + v_y, v_x) + delta;
            alpha_r =  atan2(dyaw * obj.p.l_r - v_y, v_x);
            % front/rear tire lateral force
            F_fy = obj.p.Df * sin(obj.p.Cf * atan(obj.p.Bf * alpha_f));
            F_ry = obj.p.Dr * sin(obj.p.Cr * atan(obj.p.Br * alpha_r));
            % rear tire longitudinal force
            F_rx = (obj.p.Cm1 - obj.p.Cm2) * t - obj.p.Cr0 - obj.p.Cr2*v_x^2;

            %% ODE
            dX = [
                v_x * cos(yaw) - v_y * sin(yaw);
            	v_x * sin(yaw) + v_y * cos(yaw);
            	1/obj.p.m * (F_rx - F_fy * sin(delta) + obj.p.m * v_y * dyaw);
            	1/obj.p.m * (F_ry + F_fy * cos(delta) - obj.p.m * v_x * dyaw);
            	dyaw;
            	1/obj.p.Iz * (F_fy * obj.p.l_f * cos(delta) - F_ry * obj.p.l_r)];
        end
    end
end