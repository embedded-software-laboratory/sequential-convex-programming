classdef SingleTrackWAccelerationController < model.vehicle.SingleTrack
    % extending Model with Acceleration Controller
    %
    % controller on basis of SL Paper/Janis' Bachelor Thesis
    properties
        torque_I
        torque_error_prev
        phi_I
        phi_error_prev
        
        u_1_ST_prev
    end

    methods
        function obj = SingleTrackWAccelerationController(Hp, dt, p)
            % call superclass constructor
            obj@model.vehicle.SingleTrack(Hp, dt, p)

            obj.torque_I = 0;
            obj.torque_error_prev = 0;
            obj.phi_I = 0;
            obj.phi_error_prev = 0;
            
            obj.u_1_ST_prev = zeros(1, obj.nu);
            
            warning('Acceleration controller for linear model is experimental, use with caution')
        end
            

        function u_1_ST = controller(obj, x_0, u_1_lin)
            % acceleration controller: converting from desired acceleration
            % to single-track inputs (namely torque and steering angle)
            %   global [a_x a_y] to [delta torque] input
            %
            % parametrization form SL paper (F1 car 1:1 scale)
            %
            % Inputs
            %   u_1_lin is controller output (global acceleration values)
            %   u_1_ST_prev is previous ST output (for this specific
            %       vehicle with given x_0 and u_1_lin

            %% Parameters
            v_long = x_0(3);
            v_lat  = x_0(4);
            yaw    = x_0(5);
            
            % get current derivative if last inputs are applied
            dX = obj.ode(x_0, obj.u_1_ST_prev);
            a_long_prev = dX(3);
            a_lat_prev = dX(4);

            v = norm([v_long, v_lat]);
            % scale v > 1 so controller works properly
            v = 90/obj.p.bounds(2,3) * v;

            % convert global to vehicle reference frame acceleration
            a_local = model.vehicle.vector_global2local(u_1_lin, yaw);
            a_long_ref = a_local(1);
            a_lat_ref = a_local(2);

            %% Torque
            [torque, obj.torque_I, obj.torque_error_prev] = utils.PID(...
                0.01, 1, 0, obj.dt, a_long_ref, a_long_prev, obj.torque_I, obj.torque_error_prev);
            % adapted bounds to match vehicle model
            %torque = max(-2, torque); % lower bound
            %torque = min( 1, torque); % upper bound
            %if torque < 0; torque = .5*torque; end % brake gain
            torque = max(obj.p.bounds(1, obj.nx + 2), torque); % lower bound
            torque = min(obj.p.bounds(2, obj.nx + 2), torque); % upper bound
           
            %% Delta
            % adapt a_lat_ref for low speeds
            % v_low scaling was: v * 0.1 => v_0 = 10
            % SL car v_max about 90m/s -> scale to current v_max
            %v_0_rescaled_for_scale = 10/90 * obj.p.bounds(2,3);
            v_0_rescaled_for_scale = 10;
            scale_v_low = v/v_0_rescaled_for_scale; 
            scale_v_low = max(0, scale_v_low); % lower bound
            scale_v_low = min(1, scale_v_low); % upper bound
            
            % considering equation a_{lat} = v^2 * tan(\delta) / L,
            %   introduce variable \phi = \delta max(v, v_0)^2
            %       (with v > v_0 and tan(\delta) = delta)
            a_lat_ref_adapted = a_lat_ref * scale_v_low;
            [phi, obj.phi_I, obj.phi_error_prev] = utils.PID(...
                2, 2000, 0, obj.dt, a_lat_ref_adapted, a_lat_prev, obj.phi_I, obj.phi_error_prev);
                %20, 2000, 0, obj.dt, a_lat_ref_adapted, a_lat_prev, obj.phi_I, obj.phi_error_prev);
            phi = max(-10000, phi); % lower bound
            phi = min( 10000, phi); % upper bound

            % NOTE thesis says using v_0 as lower bound - but in simulink
            %   using 2 instead of 10 (as above)
            %   scaling done as above
            %v_0_rescaled_for_delta = 2/90^2 * obj.p.bounds(2,3);
            v_0_rescaled_for_delta = 2;
            delta = phi/max(v_0_rescaled_for_delta, v)^2;
            % scale delta because of strange outputs to CarMaker [-2, 2]
            delta = (obj.p.bounds(2, obj.nx + 1)-obj.p.bounds(1, obj.nx + 1))/(2--2) * delta;
            % adapted bounds to match vehicle model
            delta = max(obj.p.bounds(1, obj.nx + 1), delta); % lower bound
            delta = min(obj.p.bounds(2, obj.nx + 1), delta); % upper bound

            %% Output
            u_1_ST = [delta, torque];
            obj.u_1_ST_prev = u_1_ST;

            fprintf([...
                '\taccel_ctrl: (with v_long %.2f, v_lat %.2f, yaw %.2f)\n'...
                '\t\t(a_x, a_y) (%.2f, %.2f) -> (a_long, a_lat) (%.2f, %.2f)-> delta %.2f, torque %.2f\n'...
                '\t\t I_torque %.2f error_torque_prev %.2f I_phi %.2f error_phi_prev %.2f\n'],...
                v_long, v_lat, yaw,...
                u_1_lin(1), u_1_lin(2), a_local(1), a_local(2), u_1_ST(1), u_1_ST(2),...
                obj.torque_I, obj.torque_error_prev, obj.phi_I, obj.phi_error_prev);
         end
    end
end