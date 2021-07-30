classdef DashboardAcceleration < plot.Base
    methods
        function plot(obj, cfg, ws)
            i_vehicle = 1;
            colors = utils.getRwthColors(100);
            color = colors(i_vehicle, :); % need to store color for later plot updates (else Matlab's gc will delete)
                
            
            
            % only plot for linear models (having accelerations as input)
            if ~cfg.scn.vhs{i_vehicle}.isControlModelLinear; return; end
            
            set(groot,'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            U_opt = ws.vhs{i_vehicle}.U_opt;
                
            % plot base track initially
            if ~obj.is_background_plotted
                clf(obj.figure_handle)
                set(obj.figure_handle,'color',[1 1 1]);
                set(obj.figure_handle,'Name', ['Dashboard: Acceleration of Vehicle ' num2str(i_vehicle)]);
                hold on
                box on

                modelParams_controller = cfg.scn.vhs{i_vehicle}.modelParams_controller;
                u_max = max([max(modelParams_controller.a_backward_max) max(modelParams_controller.a_forward_max) max(modelParams_controller.a_lateral_max)]);

                a = linspace(0,2*pi,50);
                c = cos(a);
                s = sin(a);

                for k = 5:5:u_max
                    plot(c*k,s*k,'-.k');
                end

                plot(u_max*[-1 1],[0 0],'k')
                plot([0 0],u_max*[-1 1],'k')
                
                daspect([1 1 1])
                xlim(u_max*[-1 1])
                ylim(u_max*[-1 1])
                xlabel('$a_x [m s^{-2}]$','interpreter','LaTeX');
                ylabel('$a_y [m s^{-2}]$','interpreter','LaTeX');

                obj.is_background_plotted = true;
            else
                obj.clear_tmp()
            end
            
            obj.add_tmp_handle(plot(U_opt(1,:),U_opt(2,:),'*-', 'Color', color));
            obj.add_tmp_handle(plot(U_opt(1,1),U_opt(2,1),'o', 'MarkerSize',10, 'MarkerEdgeColor','k', 'MarkerFaceColor',color));
        end
    end
end