classdef DashboardAcceleration < plot.Base
    %RACE Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        function plot(obj, cfg, ws)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % TODO only using vehicle 1
            i_vehicle = 1;
            colors = utils.getRwthColors(100);
            color = colors(i_vehicle, :); % need to store color for later plot updates (else Matlab's gc will delete)
                
            
            
            % only plot for linear models (having accelerations as input)
            if ~cfg.scn.vs{i_vehicle}.isModelLinear; return; end
            
            set(groot,'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            U_opt = ws.vs{i_vehicle}.U_opt;
                
            % plot base track initially
            if ~obj.is_background_plotted
                clf(obj.figure_handle)
                set(obj.figure_handle,'color',[1 1 1]);
                set(obj.figure_handle,'Name', ['Dashboard: Acceleration of Vehicle ' num2str(i_vehicle)]);
                hold on
                box on

                model_p = cfg.scn.vs{i_vehicle}.model_p;
                u_max = max([max(model_p.a_backward_max) max(model_p.a_forward_max) max(model_p.a_lateral_max)]);

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