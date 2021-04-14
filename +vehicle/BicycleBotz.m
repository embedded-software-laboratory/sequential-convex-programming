classdef BicycleBotz < vehicle.BaseOde
    % input: /delta steering angle, /tau torque rear wheels
    methods
        function obj = BicycleBotz(p)
            obj@vehicle.BaseOde(p, 6, 2) % call superclass constructor
        end
        
        function dX = ode(obj, X, u)
            obj.p.idx_position_x = 1;  % x-position of vehicle CoG in intertial frame
            obj.p.idx_position_y = 2;  % y-position of vehicle CoG in intertial frame
            obj.p.idx_velocity_x = 3;  % longitudinal velocity of vehicle CoG
            obj.p.idx_velocity_y = 4;  % lateral velocity of vehicle CoG
            obj.p.idx_yaw = 5;         % vehicle angle relative to inertial frame (yaw)
            obj.p.idx_dyaw = 6;        % yaw rate
            
            % bicycle ODE

            % The kinetic bicycle model from Liniger et al. (2014) is used and
            % slightly adapted to represent the vehicle dynamics. The model used in
            % Liniger et al. (2014) is based on Velenis et al. (2009) and Voser et
            % al. (2010), tire forces are modeled based on a simplified Pacejka
            % Tire Model presented in Bakker et al. (1987). The longitudinal force
            % of the rear wheel (Fr_x) is simplified.

            % Inputs:   p (parameter struct), X (vector of current system states),
            %           u (vector of current system inputs)
            % Ouputs:   dX (vector of first order differential equations)

            %% States
            x       = X(obj.p.idx_position_x);  % x-position of vehicle CoG in intertial frame
            y       = X(obj.p.idx_position_y);  % y-position of vehicle CoG in intertial frame
            dx      = X(obj.p.idx_velocity_x);  % longitudinal velocity of vehicle CoG
            dy      = X(obj.p.idx_velocity_y);  % lateral velocity of vehicle CoG
            yaw     = X(obj.p.idx_yaw);         % vehicle angle relative to inertial frame (yaw)
            dyaw    = X(obj.p.idx_dyaw);        % yaw rate

            %% Inputs
            steering_angle = u(obj.p.idx_steering_angle);
            motor_torque = u(obj.p.idx_motor_torque);

            %% Differential equations
            alpha_f = -atan((dyaw * obj.p.vehicleModel_Lf + dy) / dx) + steering_angle;    % front tire slip angle
            alpha_r = atan((dyaw * obj.p.vehicleModel_Lr - dy) / dx);                      % rear tire slip angle

            F_fy = obj.p.tireModel_Df * sin(obj.p.tireModel_Cf * atan(obj.p.tireModel_Bf * alpha_f)); % front tire lateral force
            F_ry = obj.p.tireModel_Dr * sin(obj.p.tireModel_Cr * atan(obj.p.tireModel_Br * alpha_r)); % rear tire lateral force
            F_rx = motor_torque * obj.p.vehicleModel_N;                                       % rear tire longitudinal force

            d_x = dx * cos(yaw) - dy * sin(yaw);
            d_y = dx * sin(yaw) + dy * cos(yaw);
            d_dx = 1/obj.p.vehicleModel_m * (F_rx - F_fy * sin(steering_angle) + obj.p.vehicleModel_m * dy * dyaw);
            d_dy = 1/obj.p.vehicleModel_m * (F_ry + F_fy * cos(steering_angle) - obj.p.vehicleModel_m * dx * dyaw);
            d_yaw = dyaw;
            d_dyaw = 1/obj.p.vehicleModel_Iz * (F_fy * obj.p.vehicleModel_Lf * cos(steering_angle) - F_ry * obj.p.vehicleModel_Lr);

            %% Pack derivative vector
            dX = X; % copy to get same shape
            dX(obj.p.idx_position_x) = d_x;
            dX(obj.p.idx_position_y) = d_y;
            dX(obj.p.idx_velocity_x) = d_dx;
            dX(obj.p.idx_velocity_y) = d_dy;
            dX(obj.p.idx_yaw)        = d_yaw;
            dX(obj.p.idx_dyaw)       = d_dyaw;
        end

        function x_initial = get_x_initial(obj)
            delta0 = 0;
            vel0 = 5;
            Psi0 = 0;
            dotPsi0 = 0;
            beta0 = 0;
            sy0 = 0;
            % initial state for simulation
            x_initial = [0,sy0,delta0,vel0,Psi0,dotPsi0,beta0]; 

            x_initial = init_STD(x_initial, obj.p)';
        end
    end
end