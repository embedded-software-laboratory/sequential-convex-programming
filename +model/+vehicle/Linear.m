classdef Linear < model.vehicle.Base
    % Linear vehicle model
    % Controls (in global coordinates):
    %   a_x acceleration longitudinal
    %   a_y acceleration lateral
    %
    % Static bounds not useful due to global coordinate system

    
    properties
        Ad
        Bd
    end
    
    methods (Static, Access = private)
        function p = getParamsDefault(dt)
            % CarMaker model from B. Alrifaee and J. Maczijewski, ‘Real-time Trajectory optimization for Autonomous Vehicle Racing using Sequential Linearization’, in 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483, doi: 10.1109/IVS.2018.8500634.
            % Linear model: x1 = Ax + Bu
            ddt = dt^2 / 2;
            p.paramsName = 'Default Linear';
            
            p.Ad = [      % global coordinates
                1 0 dt 0; % p_x
                0 1 0 dt; % p_y
                0 0 1 0;  % v_x
                0 0 0 1]; % v_y
            
            p.Bd = [
            %   a_x a_y (globally)
                ddt 0;
                0   ddt;
                dt  0;
                0   dt];
        end
    end
    
    methods
        function obj = Linear(Hp, dt, p)
            obj@model.vehicle.Base(4, 2, Hp, dt, p) % call superclass constructor
        end
            
        function dx = ode(obj, x, u)
            dx = (obj.p.Ad - eye(size(obj.p.Ad))) * x + obj.p.Bd * u;
        end
        
        function [Ad, Bd, Ed] = calculatePredictionMatrices(obj, ~, ~)
            % as model is linear, linearization doesn't depend on working
            % point. Thus, additional arguments not required
            
            % expand linear model to all prediction steps
            Ad = repmat(obj.p.Ad, 1, 1, obj.Hp);
            Bd = repmat(obj.p.Bd, 1, 1, obj.Hp);
            Ed = repmat(zeros(obj.nx, 1), 1, 1, obj.Hp);
        end
    end
    
    
    methods (Static)
        function p = getParamsF1CarMaker(dt, shallPlot)
            % recorded from CarMaker's Formula One vehicle (see B. Alrifaee and J. Maczijewski, “Real-time Trajectory optimization for Autonomous Vehicle Racing using Sequential Linearization,” in 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483. doi: 10.1109/IVS.2018.8500634.
            
            if ~exist('shallPlot', 'var'); shallPlot = false; end
            
            p = model.vehicle.Linear.getParamsDefault(dt);
            p.paramsName = 'Linear Empirically Car Maker 1:1';
            
            % Empirically determined maximum accelerations in [m/s^2]in the forwards, backwards
            % and lateral directions, for varying speeds in [m/s]
            v_measured = 0:0.01:120;
            p.a_lateral_max  = interp1([0 10 43 52 200], [1 14 28 33 33], v_measured);
            p.a_forward_max  = interp1([0 10 20 35 52 79 83 200],[2 13 18 18 15 6 1 1], v_measured);
            p.a_backward_max = interp1([0 30 40 52 76 200],[11 13 24 30 40 40], v_measured);

            if shallPlot
                figure
                plot(v_measured, p.a_lateral_max, '-', ...
                     v_measured, p.a_forward_max, '--', ...
                     v_measured, p.a_backward_max, ':');
                legend('$a_{lateral,max}$', '$a_{forward,max}$', '$a_{backward,max}$', 'interpreter', 'latex');
                xlabel('$v [\frac{m}{s}]$', 'interpreter', 'latex');
                ylabel('$a[\frac{m}{s^2}]$', 'interpreter', 'latex');
                
                disp('Pausing, press key to continue..'); pause
            end
            
            p.a_max = max([
                p.a_backward_max, p.a_forward_max, p.a_lateral_max]);

            % get index for v in [m/s], restrict to measured velocities
            p.v_idx = @(v) min(12001, max(1, round(100*v + 1)));
        end
        
        
        function p = getParamsSingleTrackLiniger(dt, filename, shallPlot)
            % calculates and saves a_maxs from Liniger's Single-Track
            % model. Load pre-calculated if file existing
            
            if ~exist('shallPlot', 'var'); shallPlot = false; end
            
            p = model.vehicle.Linear.getParamsDefault(dt);
            p.paramsName = 'Linear from Simulation ST Liniger 1:43';

            % debug with calculate_a_max(true)
            % production usage with get_a_max(<filename>)
            if ~isfile(filename)
                a_all_max = calculate_a_max(shallPlot);
                a_forward_max = a_all_max(1, :);
                a_backward_max = a_all_max(2, :);
                a_lateral_max = a_all_max(3, :);
                v_measured = a_all_max(4, :);
                save(filename, 'a_forward_max', 'a_backward_max', 'a_lateral_max', 'v_measured');
            else
                load(filename); %#ok<LOAD>
            end
            
            if shallPlot
                figure
                plot(v_measured, a_lateral_max, '-', ...
                     v_measured, a_forward_max, '--', ...
                     v_measured, a_backward_max, ':');
                legend('$a_{lateral,max}$', '$a_{forward,max}$', '$a_{backward,max}$', 'interpreter', 'latex');
                xlabel('$v [\frac{m}{s}]$', 'interpreter', 'latex');
                ylabel('$a[\frac{m}{s^2}]$', 'interpreter', 'latex');
                
                disp('Pausing, press key to continue..'); pause
            end
            
            p.a_forward_max = a_forward_max;
            p.a_backward_max = abs(a_backward_max); %CAVE positive values are required
            p.a_lateral_max = a_lateral_max;
            
            p.a_max = max([
                p.a_backward_max, p.a_forward_max, p.a_lateral_max]);
            
            % get index for v in [m/s], restrict to measured velocities
            % CAVE: match 100 with step size of function below
            p.v_idx = @(v) min(length(v_measured), max(1, round(100*v + 1)));

            function output_ = calculate_a_max(shallPlot, mode, v_max, v, delta)
                % runs all tests acc. to SL paper: call without any arguments
                %
                % recursively calls itself with arguments; starting w/o arguments with
                % mode longitudinal forward

                %% Config
                if ~exist('shallPlot', 'var'); shallPlot = false; end
                if ~exist('mode', 'var'); mode = 10; end
                % mode
                mode_lat = 20;
                mode_long_forward = 10; mode_long_backward = 11;

                dt_sim = 0.01; % [s]
                t_end = 20; % [s]
                n_points_steady_state = 50; % [-] number of steady points to cancel simulation

                %% Simulate
                t_end_index = t_end / dt_sim;
                Hp = 1; % arbitrarym not used here

                m_params = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
                m = model.vehicle.SingleTrack(Hp, dt_sim, m_params);

                t_sim = (0:t_end_index) * dt_sim;

                x = NaN(length(t_sim), 6);
                switch mode
                    case mode_long_forward
                        x(1, :) = [0 0 0 0 0 0]; % initial state for max power
                    case  mode_long_backward
                        x(1, :) = [0 0 v_max 0 0 0]; % initial state for braking
                    case mode_lat
                        x(1, :) = [0 0 v 0 0 0]; % arbitrary velocity, CAVE: match with below
                end

                u = NaN(length(t_sim), 2);
                switch mode
                    case mode_long_forward
                        u(:, 1) = 0; % delta
                        u(:, 2) = m_params.bounds(2, end); % torque
                    case mode_long_backward
                        u(:, 1) = 0; % delta
                        u(:, 2) = m_params.bounds(1, end); % torque
                    case mode_lat
                        % Increasing steering angle in 2nd half of simulation -> first
                        % reach steady state of velocity
                        u(:, 1) = delta;
                        u(:, 2) = 0; % torque
                end


                a_measured = NaN(length(t_sim), 2); % a_x a_y


                for i = 2:length(t_sim)
                    % simulate model from last timestep for current
                    [~, x_sim] = ode15s(@(t, x) m.ode(x, u(i, :)), [t_sim(i - 1) t_sim(i)], x(i - 1, :));
                    x(i, :) = x_sim(end, :);

                    switch mode
                        case mode_long_forward
                            % stop simulation if reached steady state
                            if sum(round(x(1:i, 3), 4) == round(x(i, 3), 4)) >= n_points_steady_state 
                                fprintf('stopped longitudinal forward at %ith step: reached steady state\n', i)
                                break
                            end
                        case mode_long_backward
                            % stop simulation if vehicle has stopped
                            if x(i, 3) < 0
                                fprintf('stopped longitudinal backward at %ith step: reached stillstand\n', i)
                                break
                            end
                        case mode_lat
                            % CAVE: setting velocity to keep steady state
                            x(i, 3) = v;
                            % stop simulation if reached steady state
                            if sum(round(x(1:i, 4), 4) == round(x(i, 4), 4)) >= n_points_steady_state 
                                fprintf('stopped lateral at %ith step: reached steady state\n', i)
                                break
                            end
                    end

                    a_measured(i, :) = (x(i, 3:4) - x(i-1, 3:4)) / dt_sim;
                end

                %CAVE assuming can extrapolate acceleration to first timestep
                a_measured(1, :) = a_measured(2, :);

                %% Plot
                if shallPlot && ~(mode == mode_lat)
                    figure('Name', 'v');
                    plot(t_sim, x(:, 3:4))
                    xlabel('t [s]');
                    legend('v_x', 'v_y');

                    figure('Name', 'a (measured)');
                    plot(t_sim, a_measured)
                    xlabel('t [s]');
                    legend('a_x', 'a_y');

                    figure('Name', 'a (measured) over v_x');
                    plot(x(:, 3), a_measured)
                    xlabel('v_x [m/s]');
                    legend('a_x', 'a_y');

                    figure('Name', 'Position');
                    plot(x(:, 1), x(:, 2))
                    xlabel('x [m]'); ylabel('y [m]');

                    disp('Pausing, press key to continue..'); pause
                end

                %% Output
                switch mode
                    case mode_long_forward
                        v_max = round(max(x(:, 3)), 4);
                        v_measure_ = 0:0.01:v_max;

                        % find changing velocity indices only 
                        i_velocity_changing = 1:(find(round(x(:, 3), 4) == v_max, 1));

                        a_forward_max_ = interp1(x(i_velocity_changing, 3), a_measured(i_velocity_changing, 1), v_measure_);

                        %% longitudinal backwards (recursive call)
                        a_backward_max_ = calculate_a_max(shallPlot, 11, v_max, v_measure_);

                        %% lateral map creation (recursive calls)
                        delta_max = m_params.bounds(2, end-1);
                        % measure loop for steady state
                        vs = [0:0.5:v_max v_max];
                        deltas = 0:0.05:delta_max;
                        a_lateral_max_all = NaN(size(vs, 1), size(deltas, 1));
                        for v = 1:length(vs)
                            for d = 1:length(deltas)
                                a_lateral_max_all(v, d) = calculate_a_max(shallPlot, 20, NaN, vs(v), deltas(d));
                            end
                        end

                        if shallPlot
                            figure('Name', 'a_y over velocity & delta input')
                            surface(deltas, vs, a_lateral_max_all(:, :))
                            xlabel('delta'); ylabel('v_x'); zlabel('a_y')
                            view(3)

                            disp('Pausing, press key to continue..'); pause
                        end
                        % CAVE simplistic approach to select fixed steering angle for
                        % finding max a_y (ok for now because of tire model not beeing
                        % non-convex)
                        [~, i_delta] = max(sum(a_lateral_max_all));
                        fprintf('using column with delta %f for a_y_max extraction\n', deltas(i_delta));
                        a_lateral_max_ = interp1(vs, a_lateral_max_all(:, i_delta)', v_measure_);

                        output_ = [a_forward_max_; a_backward_max_; a_lateral_max_; v_measure_];

                        if shallPlot
                            figure('Name', 'a_max over v_x');
                            plot(v_measure_, output_)
                            xlabel('v_x [m/s]'); ylabel('a [m/s^2]')
                            legend('$a_{forward,max}$', '$a_{backward,max}$', '$a_{lateral,max}$', 'interpreter', 'latex');

                            disp('Pausing, press key to continue..'); pause
                        end
                    case mode_long_backward
                        v_measure_ = 0:0.01:v_max;
                        % find positive velocity indices only (except last one, will be
                        % overwritten below
                        i_positive_velocity = 1:(find(x(:, 3) < 0, 1));
                        % CAVE assuming standstill has same acceleration as last data point
                        % before
                        x(i_positive_velocity(i), 3) = 0;
                        a_measured(i_positive_velocity(i), 1) = a_measured(i_positive_velocity(i - 1), 1);

                        output_ = interp1(x(i_positive_velocity, 3), a_measured(i_positive_velocity, 1), v_measure_);
                    case mode_lat
                        % a_y = yaw_rate * v_x at steady state (last entry)
                        output_ = x(i, 6) * x(i, 3);
                        fprintf('for v_x %f & delta %f got a_y %f\n', v, delta, output_)
                end
            end
        end
    end
end