classdef SingleTrackWAccelerationController < model.vehicle.SingleTrack
    % extending Model with Acceleration Controller
    properties
        vehicle_model

        torque_I
        torque_error
        phi_I
        phi_error
    end

    methods
        function obj = SingleTrackWAccelerationController(Hp, dt, p)
            % call superclass constructor
            obj@model.vehicle.SingleTrack(Hp, dt, p)

            obj.torque_I = 0;
            obj.torque_error = 0;
            obj.phi_I = 0;
            obj.phi_error = 0;
        end

        function dX = ode(obj, x, u)
            % Inputs
            %   dt  [s] simulation dt (not vehicle controller dt!)
            % convert global [a_x a_y] to [delta torque] input
            u = obj.controller(x, u);

            % call actual vehicle's (that is superclass's) ODE
            dX = ode@SingleTrack(obj, x, u);
        end

        function u_1_ST = controller(obj, x_0, u_1_lin)
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

            v = norm([v_long, v_lat]);

            % convert global to vehicle reference frame
            a_local = model.vehicle.vector_global2local(u_1_lin, yaw);
            a_long_ref = a_local(1);
            a_lat_ref = a_local(2);

            %% Torque
            [torque, obj.torque_I, obj.torque_error] = utils.PID(...
                0.01, 1, 0, obj.dt_simulation, a_long_ref, a_long_actual, obj.torque_I, obj.torque_error);
            % saturation
            torque = max(-2, torque); % lower bound
            torque = min( 1, torque); % upper bound
            if torque < 0; torque = .5*torque; end

            %% Delta
            % adapt a_lat_ref for low speeds
            a_lat_ref_adapted = a_lat_ref * min(1, v/v_0);
            [phi, obj.phi_I, obj.phi_error] = utils.PID(...
                20, 2000, 0, obj.dt_simulation, a_lat_ref_adapted, a_lat_actual, obj.phi_I, obj.phi_error);
            phi = max(-10000, phi); % lower bound
            phi = min( 10000, phi); % upper bound

            v_adapted = v;
            v_adapted = max(-2, v_adapted); % lower bound
            delta = v_adapted^2/phi;
            delta = max(-2, delta); % lower bound
            delta = min( 2, delta); % upper bound


            %% Output
            u_1_ST = [delta, torque];

            fprintf('accel_ctrl: (with v_long %.2f, v_lat %.2f, yaw %.2f, dyaw %.2f)\n\t(a_x, a_y) (%.2f, %.2f) -> (a_long, a_lat) (%.2f, %.2f)-> delta %.2f, torque %.2f\n',...
                v_long, v_lat, yaw, dyaw,...
                u_1_lin(1), u_1_lin(2), a_local(1), a_local(2), u_1_ST(1), u_1_ST(2));
         end
    end
end