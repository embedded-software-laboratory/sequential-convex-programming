classdef DashboardStatesNInputs < plot.Base
    %RACE Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        function plot(obj, scn, ws)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
    
            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            %% Prepare data
            x = ws.vs{1}.x;
            x = [ws.vs{1}.x0 x];

            u = ws.vs{1}.u;
            u = [u u(:,end)]; % Duplicate last entry for better visibility in plot

            Hp = size(ws.vs{1}.x,2);
            Tx = [0:1:Hp];
            Tu = [1:(Hp+1)]; % include Hp+1 to display the values at Hp as one stair step
            
            %% plot base track initially
            if ~obj.is_background_plotted
                clf(obj.figure_handle)
                set(obj.figure_handle,'Name','Dashboard: States & Inputs');
                hold on
                
                % create plots
                obj.subplot_handles{1} = subplot(3,2,1);
                title('v_x [m/s]')
                ylim([-3 3])
                grid on

                obj.subplot_handles{2} = subplot(3,2,2);
                title('v_y [m/s]')
                ylim([-0.7 0.7])
                grid on

                %if length(x(:, 1)) >=6 % TODO make state dependent
                    obj.subplot_handles{3} = subplot(3,2,3);
                    title('Yaw Angle Phi [rad]')
                    ylim([-3*pi 3*pi])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi','-2\pi','-\pi','0','\pi','2\pi','3\pi'})
                    grid on

                    obj.subplot_handles{4} = subplot(3,2,4);
                    title('Yaw Rate W [rad/sec]')
                    ylim([-3.5*pi 3.5*pi])
                    yticks([-3*pi -2*pi -pi 0 pi 2*pi 3*pi])
                    yticklabels({'-3\pi/s','-2\pi/s','-\pi/s','0','\pi/s','2\pi/s','3\pi/s'})
                    grid on
                %end

                obj.subplot_handles{5} = subplot(3,2,5);
                title('Input Steering Angle [rad]')
                ylim([-0.4 0.4])
                grid on

                obj.subplot_handles{6} = subplot(3,2,6);
                title('Input Torque [Nm]')
                ylim([-0.12 0.12])
                grid on

                obj.is_background_plotted = true;
                
                obj.subplot_handles = get(gcf, 'Children');
            else
                obj.clear_tmp()
                cla;
            end


            %a = subplot(obj.subplot_handles{1});
            %a = axes;
            obj.add_tmp_handle(plot(obj.subplot_handles(1), Tx,x(3,:)))

            %subplot(3,2,2);
            obj.add_tmp_handle(plot(obj.subplot_handles(2), Tx,x(4,:)))

            if length(x(:, 1)) >=6 % TODO make state dependent
                %subplot(3,2,3);
                obj.add_tmp_handle(plot(obj.subplot_handles(3), Tx,x(5,:)))

                %subplot(3,2,4);
                obj.add_tmp_handle(plot(obj.subplot_handles(4), Tx,x(6,:)))
            end

            %subplot(3,2,5);
            obj.add_tmp_handle(stairs(obj.subplot_handles(5), Tu,u(1,:)))

            %subplot(3,2,6);
            obj.add_tmp_handle(stairs(obj.subplot_handles(6), Tu,u(2,:)))
        end
    end
end

