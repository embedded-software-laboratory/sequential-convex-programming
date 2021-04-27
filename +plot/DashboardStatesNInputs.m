classdef DashboardStatesNInputs < plot.Base
    %RACE Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        function plot(obj, ~, ws)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
    
            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            %% Prepare data
            x = ws.vs{1}.x;
            x = [ws.vs{1}.x0 x];

            u = ws.vs{1}.u;
            u = [u u(:,end)]; % Duplicate last entry for better visibility in plot

            Hp = size(ws.vs{1}.x,2);
            Tx = 0:1:Hp;
            Tu = 1:(Hp + 1); % include Hp+1 to display the values at Hp as one stair step
            
            %% plot base track initially
            if ~obj.is_background_plotted
                clf(obj.figure_handle)
                set(obj.figure_handle, 'Name', 'Dashboard: States & Inputs');
                hold on
                
                % create plots
                subplot(3,2,1);
            	obj.subplot_plot_handles{1} = plot(Tx, x(3,:));
                title('v_x [m/s]')
%                 ylim([-3 3])
                xlim([Tx(1) Tx(end)])
                grid on

                subplot(3,2,2);
                obj.subplot_plot_handles{2} = plot(Tx, x(4,:));
                title('v_y [m/s]')
%                 ylim([-0.7 0.7])
                xlim([Tx(1) Tx(end)])
                grid on

                if length(x(:, 1)) >=6 % TODO make state dependent
                    subplot(3,2,3);
                    obj.subplot_plot_handles{3} = plot(Tx, x(5,:));
                    title('Yaw Angle Phi [rad]')
                    ylim([-3*pi 3*pi])
                    xlim([Tx(1) Tx(end)])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi','-2\pi','-\pi','0','\pi','2\pi','3\pi'})
                    grid on

                    subplot(3,2,4);
                    obj.subplot_plot_handles{4} = plot(Tx, x(6,:));
                    title('Yaw Rate W [rad/sec]')
                    ylim([-3.5*pi 3.5*pi])
                    xlim([Tx(1) Tx(end)])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi/s','-2\pi/s','-\pi/s','0','\pi/s','2\pi/s','3\pi/s'})
                    grid on
                end

                subplot(3,2,5);
                obj.subplot_plot_handles{5} = plot(Tu, u(1,:));
                title('Input Steering Angle [rad]')
%                 ylim([-0.4 0.4])
                xlim([Tu(1) Tu(end)])
                grid on

                subplot(3,2,6);
                obj.subplot_plot_handles{6} = plot(Tu, u(2,:));
                title('Input Torque [Nm]')
%                 ylim([-0.12 0.12])
                xlim([Tu(1) Tu(end)])
                grid on

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
    end
end

