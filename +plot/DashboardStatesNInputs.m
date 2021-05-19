classdef DashboardStatesNInputs < plot.Base
    %RACE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        table_desc
        table_val
    end
    
    methods
        function obj = DashboardStatesNInputs(figure_handle_number)
            % call superclass constructor
            obj@plot.Base(figure_handle_number)
            
            obj.table_desc = {};
            obj.table_val = {};
        end
            
        function plot(obj, cfg, ws)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
    
            % choose vehicle
            i_veh = 1;
            colors = utils.getRwthColors(100);
            color = colors(i_veh, :); % need to store color for later plot updates (else Matlab's gc will delete)
            
            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            %% Prepare data
            x = ws.vs{i_veh}.x;
            x = [ws.vs{i_veh}.x0 x];

            u = ws.vs{i_veh}.u;
            u = [u u(:,end)]; % Duplicate last entry for better visibility in plot

            Hp = size(ws.vs{i_veh}.x,2);
            Tx = 0:1:Hp;
            Tu = 1:(Hp + 1); % include Hp+1 to display the values at Hp as one stair step
            
            %% plot base track initially
            if ~obj.is_background_plotted
                clf(obj.figure_handle)
                set(obj.figure_handle, 'Name', 'Dashboard: States & Inputs');
                hold on
                
                
                % create plots
                subplot(3,3,1);
            	obj.subplot_plot_handles{1} = plot(Tx, x(3,:), 'Color', color);
                title('v_x [m/s]')
%                 ylim([-3 3])
                xlim([Tx(1) Tx(end)])
                grid on

                subplot(3,3,2);
                obj.subplot_plot_handles{2} = plot(Tx, x(4,:), 'Color', color);
                title('v_y [m/s]')
%                 ylim([-0.7 0.7])
                xlim([Tx(1) Tx(end)])
                grid on

                if length(x(:, 1)) >=6 % TODO make state dependent
                    subplot(3,3,4);
                    obj.subplot_plot_handles{3} = plot(Tx, x(5,:), 'Color', color);
                    title('Yaw Angle Phi [rad]')
                    ylim([-3*pi 3*pi])
                    xlim([Tx(1) Tx(end)])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi','-2\pi','-\pi','0','\pi','2\pi','3\pi'})
                    grid on

                    subplot(3,3,5);
                    obj.subplot_plot_handles{4} = plot(Tx, x(6,:), 'Color', color);
                    title('Yaw Rate W [rad/sec]')
                    ylim([-3.5*pi 3.5*pi])
                    xlim([Tx(1) Tx(end)])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi/s','-2\pi/s','-\pi/s','0','\pi/s','2\pi/s','3\pi/s'})
                    grid on
                end

                subplot(3,3,7);
                obj.subplot_plot_handles{5} = plot(Tu, u(1,:), 'Color', color);
                title('Input Steering Angle [rad]')
                ylim([-0.4 0.4])
                xlim([Tu(1) Tu(end)])
                grid on

                subplot(3,3,8);
                obj.subplot_plot_handles{6} = plot(Tu, u(2,:), 'Color', color);
                title('Input Torque [Nm]')
                ylim([-0.12 0.12])
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
                
                
                for i = 1:length(cfg.scn.vs)
                    vh = cfg.scn.vs{i};                
                    obj.add_table_line('', '');
                    obj.add_table_line(['\bfVehicle ' num2str(i) '\rm with \bf\color[rgb]{' sprintf('%f,%f,%f', colors(i, :)) '}color\rm\color{black}'], '');
                    obj.add_table_line('Vehicle Model', class(vh.model));
                    obj.add_table_line('Vehicle Params', vh.model.p.paramsName);
                    obj.add_table_line('Vehicle Sim Model', class(vh.model_simulation));
                    obj.add_table_line('Vehicle Sim Params', vh.model_simulation.p.paramsName);
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
            set(obj.subplot_plot_handles{1}, 'YData', x(3,:));
            
            set(obj.subplot_plot_handles{2}, 'XData', Tx);
            set(obj.subplot_plot_handles{2}, 'YData', x(4,:));
            
            if length(x(:, 1)) >=6 % TODO make state dependent
                set(obj.subplot_plot_handles{3}, 'XData', Tx);
                set(obj.subplot_plot_handles{3}, 'YData', x(5,:));
                
                set(obj.subplot_plot_handles{4}, 'XData', Tx);
                set(obj.subplot_plot_handles{4}, 'YData', x(6,:));
            end

            set(obj.subplot_plot_handles{5}, 'XData', Tu);
            set(obj.subplot_plot_handles{5}, 'YData', u(1,:));
            
            set(obj.subplot_plot_handles{6}, 'XData', Tu);
            set(obj.subplot_plot_handles{6}, 'YData', u(2,:));
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

