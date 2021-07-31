classdef Dashboard < plot.Base
    properties
        table_desc
        table_val
    end
    
    methods
        function obj = Dashboard(figure_handle_number)
            % call superclass constructor
            obj@plot.Base(figure_handle_number)
            
            obj.table_desc = {};
            obj.table_val = {};
        end
            
        function plot(obj, cfg, ws)
            % choose vehicle
            i_vh = 1;
            
            % ease variable access
            vh = ws.vhs{i_vh};
            vh_cfg = cfg.scn.vhs{i_vh};
            modelParams_controller = vh_cfg.modelParams_controller;
            bounds_available = isfield(modelParams_controller, 'bounds');
            colors = utils.getRwthColors(100);
            color = colors(i_vh, :); % need to store color for later plot updates (else Matlab's gc will delete)
            
            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            %% Prepare data
            X = vh.X_controller;
            X = [vh.x_0_controller X];
            % if you want "vehicle" reference frame velocities, outcomment
            % following
            % NOTE due to linear model, no slip angle is assumed. thus
            % v_{lateral} = 0
            %if vh_cfg.isControlModelLinear
            %    % convert global to vehicle frame velocities
            %    for k = 1:length(X)
            %        X(3:4, k) = model.vehicle.velocity_global2local(X(3:4, k));
            %    end
            %end

            U = vh.U_controller;
            U = [U U(:,end)]; % Duplicate last entry for better visibility in plot

            Hp = size(vh.X_controller, 2);
            Tx = 0:1:Hp;
            Tu = 1:(Hp + 1); % include Hp+1 to display the values at Hp as one stair step
            
            %% plot base track initially
            if ~obj.is_background_plotted
                clf(obj.figure_handle)
                set(obj.figure_handle, 'Name', 'Dashboard: States & Inputs');
                hold on
                
                % create plots
                subplot(3,3,1);
            	obj.subplot_plot_handles{1} = plot(Tx, X(3,:), 'Color', color);
                if vh_cfg.isControlModelLinear
                    title('Controller: v_{x} (global) [m/s]')
                else
                    title('Controller: v_{long} [m/s]')
                end
                if bounds_available
                    ylim(modelParams_controller.bounds(:, 3)' .* 1.1)
                    yline(modelParams_controller.bounds(:, 3)', '--r')
                end
                xlim([Tx(1) Tx(end)])
                grid on

                subplot(3,3,2);
                obj.subplot_plot_handles{2} = plot(Tx, X(4,:), 'Color', color);
                if vh_cfg.isControlModelLinear
                    title('Controller: v_{y} (global) [m/s]')
                else
                    title('Controller: v_{lateral} [m/s]')
                end
                if bounds_available
                    ylim(modelParams_controller.bounds(:, 4)' .* 1.1)
                    yline(modelParams_controller.bounds(:, 4)', '--r')
                end
                xlim([Tx(1) Tx(end)])
                grid on

                if ~vh_cfg.isControlModelLinear
                    subplot(3,3,4);
                    % plot multiple parallel yaw angles:
                    %obj.subplot_plot_handles{3} = plot(Tx, [X(5,:); X(5,:) + 2*pi; X(5,:) - 2*pi], 'Color', color);
                    obj.subplot_plot_handles{3} = plot(Tx, X(5,:), 'Color', color);
                    title('Controller: Yaw Angle \phi [rad]')
                    if bounds_available
                        % if bounds of model are real
                        if ~any(isinf(modelParams_controller.bounds(:, 5)'))
                            ylim(modelParams_controller.bounds(:, 5)' .* 1.1)
                            yline(modelParams_controller.bounds(:, 5)', '--r')
                        else % use default bounds
                            bounds = [-pi, pi]; % use periodicity
                            ylim(bounds .* 1.1)
                        end
                    end
                    xlim([Tx(1) Tx(end)])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi','-2\pi','-\pi','0','\pi','2\pi','3\pi'})
                    grid on

                    subplot(3,3,5);
                    obj.subplot_plot_handles{4} = plot(Tx, X(6,:), 'Color', color);
                    title('Controller: Yaw Rate \omega [rad/sec]')
                    if bounds_available
                        ylim(modelParams_controller.bounds(:, 6)' .* 1.1)
                        yline(modelParams_controller.bounds(:, 6)', '--r')
                    end
                    xlim([Tx(1) Tx(end)])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi/s','-2\pi/s','-\pi/s','0','\pi/s','2\pi/s','3\pi/s'})
                    grid on
                end

                subplot(3,3,7);
                obj.subplot_plot_handles{5} = plot(Tu, U(1,:), 'Color', color);
                if vh_cfg.isControlModelLinear
                    title('Controller: a_{x} (global) [m/s^2]')
                else
                    title('Controller: Input Steering Angle [rad]')
                end
                if bounds_available
                    ylim(modelParams_controller.bounds(:, length(X(:, 1)) + 1)' .* 1.1)
                    % if bounds are real
                    if ~any(isinf(modelParams_controller.bounds(:, length(X(:, 1)) + 1)'))
                        yline(modelParams_controller.bounds(:, length(X(:, 1)) + 1)', '--r')
                    end
                end
                xlim([Tu(1) Tu(end)])
                grid on

                subplot(3,3,8);
                obj.subplot_plot_handles{6} = plot(Tu, U(2,:), 'Color', color);
                
                if vh_cfg.isControlModelLinear
                    title('Controller: a_{y} (global) [m/s^2]')
                else
                    title('Controller: Input Torque [% or Nm]')
                end
                if bounds_available
                    ylim(modelParams_controller.bounds(:, length(X(:, 1)) + 2)' .* 1.1)
                    % if bounds are real
                    if ~any(isinf(modelParams_controller.bounds(:, length(X(:, 1)) + 2)'))
                        yline(modelParams_controller.bounds(:, length(X(:, 1)) + 2)', '--r')
                    end
                end
                xlim([Tu(1) Tu(end)])
                grid on
                
                % Text
                subplot(3,3,[3,6,9]);
                axis off
                
                %% assemble text
                obj.add_table_line('\bfControl: press ESC to abort, SPACE to pause\rm', '');
                obj.add_table_line('', '');
                obj.add_table_line('\bfConfiguration\rm', '');
                % get name of function handle
                obj.add_table_line('Track', functions(cfg.scn.track_handle).function);
                obj.add_table_line('Track Creation Scale', utils.rat2str(cfg.scn.track_creation_scale));
                
                
                for i = 1:length(cfg.scn.vhs)
                    vh = cfg.scn.vhs{i};                
                    obj.add_table_line('', '');
                    obj.add_table_line(['\bfVehicle ' num2str(i) '\rm with \bf\color[rgb]{' sprintf('%f,%f,%f', colors(i, :)) '}color\rm\color{black}'], '');
                    obj.add_table_line('Vehicle Control Model', class(vh.model_controller));
                    obj.add_table_line('Vehicle Control Params', vh.model_controller.p.paramsName);
                    if ~cfg.scn.is_main_vehicle_only || i == 1 % only for vehicles !=1 when main vehicle simulation mode
                        obj.add_table_line('Vehicle Sim Model', class(vh.model_simulation));
                        obj.add_table_line('Vehicle Sim Params', vh.model_simulation.p.paramsName);
                    else
                        obj.add_table_line('Vehicle Sim Model', 'simulation via main vehicle #1');
                        obj.add_table_line('Vehicle Sim Params', 'simulation via main vehicle #1');
                    end
                    if vh.approximation == vh.approximationSL
                        approx = 'SL';
                    elseif vh.approximation == vh.approximationSCR
                        approx = 'SCR';
                    end
                    obj.add_table_line('Track Approximation via', approx);
                    obj.add_table_line('Blocking enabled', obj.get_logical_string(vh.p.isBlockingEnabled));
                    obj.add_table_line('Obstacles considered', obj.get_logical_string(vh.p.areObstaclesConsidered));
                end
                
                % place text top left
                text(0, 1, obj.table_desc, 'VerticalAlignment', 'top')
                text(0.6, 1, obj.table_val, 'VerticalAlignment', 'top')

                obj.is_background_plotted = true;
            else
                %obj.clear_tmp()
                %cla;
            end

            % update plots
            set(obj.subplot_plot_handles{1}, 'XData', Tx);
            set(obj.subplot_plot_handles{1}, 'YData', X(3,:));
            
            set(obj.subplot_plot_handles{2}, 'XData', Tx);
            set(obj.subplot_plot_handles{2}, 'YData', X(4,:));
            
            if length(X(:, 1)) >=6 % TODO make state dependent
                set(obj.subplot_plot_handles{3}, 'XData', Tx);
                % plot multiple parallel yaw angles:
                %set(obj.subplot_plot_handles{3}, {'YData'}, num2cell([utils.removePiPeriodicity(X(5,:)); utils.removePiPeriodicity(X(5,:)) + 2*pi; utils.removePiPeriodicity(X(5,:)) - 2*pi], 2));
                set(obj.subplot_plot_handles{3}, 'YData', utils.removePiPeriodicity(X(5,:)));
                
                set(obj.subplot_plot_handles{4}, 'XData', Tx);
                set(obj.subplot_plot_handles{4}, 'YData', X(6,:));
            end

            
            set(obj.subplot_plot_handles{5}, 'XData', Tu);
            set(obj.subplot_plot_handles{5}, 'YData', U(1,:));

            set(obj.subplot_plot_handles{6}, 'XData', Tu);
            set(obj.subplot_plot_handles{6}, 'YData', U(2,:));
        end
        
        function add_table_line(obj, desc, val)
            obj.table_desc{end + 1} = desc;
            if ~isempty(val)
                obj.table_desc{end} = [obj.table_desc{end} ':'];
            end
            obj.table_val{end + 1} = val;
        end
        
        function logical_str = get_logical_string(~, logical)
            logical_str = {'false', 'true'};
            % shift for matlab 1-indexing
            logical_str = logical_str{logical + 1};
        end
            
    end
end

