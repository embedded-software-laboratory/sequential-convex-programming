classdef DashboardAcceleration < plot.Base
    %RACE Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        function plot(obj, scn, ws)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            set(groot,'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            clf(obj.figure_handle)
            set(obj.figure_handle,'color',[1 1 1]);
            set(obj.figure_handle,'Name','Dashboard: Acceleration');
            hold on
            box on
            
            % TODO only using vehicle 1
            i_vehicle = 1;
            p = scn.vs{i_vehicle}.p;
            u = ws.vs{i_vehicle}.u;


            u_max = max([max(p.a_backward_max_list) max(p.a_forward_max_list) max(p.a_lateral_max_list)]);

            a = linspace(0,2*pi,50);
            c = cos(a);
            s = sin(a);

            for k = 5:5:u_max
                plot(c*k,s*k,'-.k');
            end

            plot(u_max*[-1 1],[0 0],'k')
            plot([0 0],u_max*[-1 1],'k')


            plot(u(1,:),u(2,:),'*-');
            plot(u(1,1),u(2,1),'o', 'MarkerSize',10, 'MarkerEdgeColor','r', 'MarkerFaceColor','r');
            daspect([1 1 1])
            xlim(u_max*[-1 1])
            ylim(u_max*[-1 1])
            xlabel('$a_x [m s^{-2}]$','interpreter','LaTeX');
            ylabel('$a_y [m s^{-2}]$','interpreter','LaTeX');
        end
    end
end