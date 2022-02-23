classdef Race < plots.Base
    methods
        function plot(obj, cfg, ws)
            scn = cfg.scn;

            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            if ~cfg.plot.grayscale
                c = utils.getRwthColors(100);
            else
                c = [0.3; 0.6] .* [1 1 1];
            end
            % plot base track initially
            if ~obj.is_background_plotted
                % clear old plot
                clf(obj.figure_handle)
                
                set(obj.figure_handle, 'color', [1 1 1]);
                set(obj.figure_handle, 'Name', ['Race (States: Position x and y) [track created with scale ' utils.rat2str(scn.track_creation_scale) ']']);
                hold on
                
                % create legend accordingly to cars drawn below
                handles = zeros(length(scn.vhs), 1);
                names = cell(length(scn.vhs), 1);
                for k = 1:length(scn.vhs)
                    % pseudo plot for coloring
                    handles(k) = fill(NaN:NaN, NaN:NaN, c(k, :));
                    names{k} = ['Vehicle ' num2str(k)];
                end
                l = legend(handles, names, 'AutoUpdate', 'off', 'Location', 'northeast');
                pos = get(l, 'Position');
            
                obj.plot_track(scn.track);
                
                set(l, 'Position', [pos(1) .9 pos(3) pos(4)]);
                
                obj.is_background_plotted = true;
            else
                obj.clear_tmp()
            end

            %% Plot vehicle specific elements
            %c = ['r','b','g','y','m','c','w','k',];
            
            for k = 1:length(scn.vhs)
                % Value asignments for better readability
                x_0_controller = ws.vhs{k}.x_0_controller;
                X_controller = ws.vhs{k}.X_controller;
                p = scn.vhs{k}.p;
                vehLength = scn.vhs{k}.lengthVal;
                vehWidth = scn.vhs{k}.widthVal;

                %% Planned trajectory
                obj.add_tmp_handle(plot(X_controller(1,:),X_controller(2,:),'.-','color',c(k, :),'MarkerSize',7));
                %% Previous trajectory
                obj.add_tmp_handle(plot(...
                    ws.vhs{k}.X_controller_prev(1,:),ws.vhs{k}.X_controller_prev(2,:),...
                    'LineStyle',':',...
                    'Marker','none',...
                    'color','black',...
                    'MarkerSize',5)...
                );

                %% Vehicle Box
                if length(x_0_controller) <= 4 % if vehicle control model is linear
                % Vehicle box (is old version: get vehicle
                % direction from current vehicle velocity vector)
                    if x_0_controller(3:4) ~= [0;0]
                        dist = x_0_controller(3:4);
                    else
                        dist = [1;0];
                    end
                    dist = dist / norm(dist);
                else
                    dist = [cos(x_0_controller(5));sin(x_0_controller(5))]; % yaw angle
                end
                R = [dist [-dist(2); dist(1)]];
                vehicleRectangle = R * [vehLength/2 0;0 vehWidth/2] * [1 1 -1 -1;1 -1 -1 1] + repmat(x_0_controller(1:2),1,4);
                obj.add_tmp_handle(fill(vehicleRectangle(1,:),vehicleRectangle(2,:),c(k, :)));
                daspect([1 1 1])

                %% Track constraints for the trajectory point at prediction/control horizon
                % as we plot after controller run, we have to find the next
                % last checkpoint/polygon (next means the one, which we
                % would adher to in next controller execution)
                if scn.vhs{k}.approximationIsSL
                    % Select checkpoint of last trajectory point
                    last_checkpoint_index = controller.track_SL.find_closest_checkpoint_index(ws.vhs{k}.X_controller_prev(1:2, p.Hp), [scn.track.center]);
                    % Select checkpoint of last trajectory point
                    last_checkpoint = scn.track(last_checkpoint_index);
                    % Length of plotted linear constraints
                    L = 30 * scn.track_creation_scale;
                    tangent_left = [(last_checkpoint.left + L* last_checkpoint.forward_vector)...
                                    (last_checkpoint.left - L* last_checkpoint.forward_vector)];
                    tangent_right = [(last_checkpoint.right + L* last_checkpoint.forward_vector)...
                                     (last_checkpoint.right - L* last_checkpoint.forward_vector)];
                    obj.add_tmp_handle(plot(tangent_left(1,:), tangent_left(2,:), '--', 'color', c(k, :), 'LineWidth', 1));
                    obj.add_tmp_handle(plot(tangent_right(1,:), tangent_right(2,:), '--', 'color', c(k, :), 'LineWidth', 1));
                else % SCR
                    last_poly_idx = controller.track_SCR.find_closest_most_forward_polygon_index(ws.vhs{k}.X_controller_prev(1:2, p.Hp), scn.track_SCR);
                    last_poly_vertices = utils.poly.cleanse_convex_polygon(...
                        utils.poly.get_track_polygon_vertices(last_poly_idx, scn.track_SCR));
                    % back-and-forth conversion to close polygon
                    obj.add_tmp_handle(plot(utils.poly.vert2poly(last_poly_vertices),...
                        'LineStyle', '--', 'EdgeColor', c(k, :), 'LineWidth', 1, ...
                        'FaceColor', c(k, :), 'FaceAlpha', 0.2));
                end

                %% Obstacle constraints for current position if obstacle has to be respected
                %     if p.obstacle_1.headingVec(2) >= 0
                %         headingObst1Angle = acos([1,0] * p.obstacle_1.headingVec / norm(p.obstacle_1.headingVec));
                %     else
                %         d = [-1 0;0 -1] * p.obstacle_1.headingVec;
                %         headingObst1Angle = acos([1,0] * d / norm(d)) + pi;
                %     end
                %     obstaclePolygon = createRectangle(...
                %        p.obstacle_1.xVal,...
                %        p.obstacle_1.yVal,...
                %        headingObst1Angle,...
                %        p.obstacle_1.lengthVal,...
                %        p.obstacle_1.widthVal);
                %     fill(obstaclePolygon(1,:),obstaclePolygon(2,:),[0 0 0]);
                
                if sum(ws.obstacleTable(k,:)) >= 1
                    % iterate over all opponents
                    for j = 1:length(scn.vhs)
                        % only if obstacles enabled
                        if scn.vhs{k}.p.areObstaclesConsidered
                            % Respect only constraints for opponents marked as obstacles
                            if scn.vhs{k}.approximationIsSL
                                if ws.obstacleTable(k,j) == 1
                                    % FIXME match plotting with shapes represented,
                                    % see createCP
                                    dist = pdist([x_0_controller(1:2)';ws.vhs{1,j}.x_0(1:2)'],'euclidean'); % calculate distance
                                    normal_vector = - (x_0_controller(1:2)-ws.vhs{1,j}.x_0(1:2))/dist; % normal vector in direction from trajectory point to obstacle center
                                    closest_obst_point = ws.vhs{1,j}.x_0(1:2) - normal_vector * scn.vhs{1,j}.distSafe2CenterVal_1; % intersection of safe radius and connection between trajectory point and obstacle center 
                                    tangent_obst = [(closest_obst_point + L * [0 -1; 1 0] * normal_vector) (closest_obst_point - L * [0 -1; 1 0] * normal_vector)];
                                    obj.add_tmp_handle(plot(tangent_obst(1,:), tangent_obst(2,:),'--','color',c(k, :),'LineWidth',1));

        %                         lastTrajectoryPoint = controller_output.x(p.idx_pos,p.Hp); % Select last trajectory point
        %                         
        %                         dist = pdist([lastTrajectoryPoint'; [p.obstacle_1.xVal p.obstacle_1.yVal]],'euclidean');            % See SL_QP
        %                         normal_vector = - (lastTrajectoryPoint-[p.obstacle_1.xVal; p.obstacle_1.yVal])/dist;                % See SL_QP
        %                         closest_obst_point = [p.obstacle_1.xVal; p.obstacle_1.yVal] - normal_vector * p.obstacle_1.distSafe2CenterVal;    % See SL_QP
        %                         
        %                         % Adjust obstacle constraint in case of deadlock
        %                         left_unit_vector = [0 -1;1 0] * last_checkpoint.forward_vector;        % See SL_QP
        %                         
        %                         if strcmp(controller_output.constrAdpt,'left')                                              % See SL_QP
        %                             normal_vector = [last_checkpoint.forward_vector'; -left_unit_vector'] * normal_vector;
        %                         elseif strcmp(controller_output.constrAdpt,'right')
        %                             normal_vector = [last_checkpoint.forward_vector'; -left_unit_vector'] * normal_vector;
        %                         end
        %                         tangent_obst = [(closest_obst_point + L * [0 -1; 1 0] * normal_vector) (closest_obst_point - L * [0 -1; 1 0] * normal_vector)];
        %                         
        %                         plot(tangent_obst(1,:), tangent_obst(2,:),'k--','LineWidth',1)
                                end
                            else
                                warning('plotting of obstacles not implemented for SCR')
                            end
                        end
                    end
                end
             end
        end
    end
    
    methods (Static)
        function plot_track(checkpoints)
            left_points = [checkpoints.left];
            right_points = [checkpoints.right];
            
            hold on

            %% Account for extra width for the vehicle
            % as widht was deducted for modelling
            %width = vehicle_width / 2;
            %normals = width*[0 -1;1 0]*forward_vectors;
            %left_points = left_points + normals;
            %right_points = right_points - normals;
            % Draw track area
            pts=[fliplr(right_points) right_points(:,end) left_points left_points(:,1)];
            fill(pts(1,:), pts(2,:),[1 1 1]*.9,'EdgeAlpha',0)

            if true
                plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)],'k','LineWidth',1);
                plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)],'k','LineWidth',1);
            else             
                % Use these commands to show single checkpoints
                plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)],'.k');
                plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)],'.k');
            end
            
            % Start / Finish Line
            plot([right_points(1,1) left_points(1,1)], [right_points(2,1) left_points(2,1)],':k')

            % Center track in figure, add padding (else will move due to
            % vehicle plotting
            bounds = [min([left_points right_points]');max([left_points right_points]')];
            xlim([bounds(1,1)-0.1 bounds(2,1)+0.1])
            ylim([bounds(1,2)-0.1 bounds(2,2)+0.1])
            xlim(mean(xlim) + diff(xlim) * [-1 1] * 0.6)
            ylim(mean(ylim) + diff(ylim) * [-1 1] * 0.6)    
            daspect([1 1 1])
            xlabel('x [m]'); ylabel('y [m]')
            
            % add scale (e.g., if no labels used)
            %axis off
            %text(0.5, .6, 'Scale: 1m', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
            %plot([0 1], [.5 .5], 'k', 'LineWidth', 1);
        end
    end
end

