classdef Race < plot.Base
    methods
        function plot(obj, cfg, ws)
            scn = cfg.scn;

            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            
            % plot base track initially
            if ~obj.is_background_plotted
                % clear old plot
                clf(obj.figure_handle)
                
                set(obj.figure_handle, 'color', [1 1 1]);
                set(obj.figure_handle, 'Name', ['Race (States: Position x and y) [track created with scale ' utils.rat2str(scn.track_creation_scale) ']']);
                hold on
                
                % create legend accordingly to cars drawn below
                c = utils.getRwthColors(100);
                handles = zeros(length(scn.vhs), 1);
                names = cell(length(scn.vhs), 1);
                for k = 1:length(scn.vhs)
                    % pseudo plot for coloring
                    handles(k) = fill(NaN:NaN, NaN:NaN, c(k, :));
                    names{k} = ['Vehicle ' num2str(k)];
                end
                l = legend(handles, names, 'AutoUpdate', 'off', 'Location', 'northeast');
                pos = get(l, 'Position');
            
                obj.plot_track(scn.track, scn.vhs{1}.widthVal)
                
                set(l, 'Position', [pos(1) .9 pos(3) pos(4)]);
                
                obj.is_background_plotted = true;
            else
                obj.clear_tmp()
            end

            %% Plot vehicle specific elements
            %c = ['r','b','g','y','m','c','w','k',];
            c = utils.getRwthColors(100);
            for k = 1:length(scn.vhs)
                % Value asignments for better readability
                x_0_controller = ws.vhs{k}.x_0_controller;
                X_controller = ws.vhs{k}.X_controller;
                p = scn.vhs{k}.p;
                vehLength = scn.vhs{k}.lengthVal;
                vehWidth = scn.vhs{k}.widthVal;

                %% Planned trajectory
                obj.add_tmp_handle(plot(X_controller(1,:),X_controller(2,:),'.-','color',c(k, :),'MarkerSize',7));    

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

                %% Draw obstacle
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

                %% Track constraints for the trajectory point at prediction/control horizon
                [~,lastCPindex] = min(sum(([scn.track.center] - repmat(X_controller(1:2,p.Hp),1,length(scn.track))).^2));
                lastCP = scn.track(lastCPindex);
                L = 1; % Length of plotted linear constraints
                tangent_left = [(lastCP.left + L* lastCP.forward_vector) (lastCP.left - L* lastCP.forward_vector)];
                tangent_right = [(lastCP.right + L* lastCP.forward_vector) (lastCP.right - L* lastCP.forward_vector)];
                obj.add_tmp_handle(plot(tangent_left(1,:), tangent_left(2,:),'--','color',c(k, :),'LineWidth',1));
                obj.add_tmp_handle(plot(tangent_right(1,:), tangent_right(2,:),'--','color',c(k, :),'LineWidth',1));
                
                %     % Constraints for the last trajectory point (point at
                %     % prediction/control horizon)
                %     if strcmp(p.name, 'SL') 
                %         last_checkpoint_index = controller_output.checkpoint_indices(end);      % Select checkpoint of last trajectory point
                %         last_checkpoint = p.checkpoints(last_checkpoint_index);                 % Select checkpoint of last trajectory point
                %         L = 30;                                                                 % Length of plotted linear constraints
                %         
                %         lastTrajectoryPoint = controller_output.x(p.idx_pos,p.Hp);                 % Select last trajectory point
                %         
                %         dist = pdist([lastTrajectoryPoint'; [p.obstacle_1.xVal p.obstacle_1.yVal]],'euclidean');            % See SL_QP
                %         normal_vector = - (lastTrajectoryPoint-[p.obstacle_1.xVal; p.obstacle_1.yVal])/dist;                % See SL_QP
                %         closest_obst_point = [p.obstacle_1.xVal; p.obstacle_1.yVal] - normal_vector * p.obstacle_1.distSafe2CenterVal;    % See SL_QP
                %         
                %         % Adjust obstacle constraint in case of deadlock
                %         left_unit_vector = [0 -1;1 0] * last_checkpoint.forward_vector;        % See SL_QP
                %         
                %         if strcmp(controller_output.constrAdpt,'left')                                              % See SL_QP
                %             normal_vector = [last_checkpoint.forward_vector'; -left_unit_vector'] * normal_vector;
                %         elseif strcmp(controller_output.constrAdpt,'right')
                %             normal_vector = [last_checkpoint.forward_vector'; -left_unit_vector'] * normal_vector;
                %         end
                %         
                %         tangent_left = [(last_checkpoint.left + L* last_checkpoint.forward_vector) (last_checkpoint.left - L* last_checkpoint.forward_vector)];
                %         tangent_right = [(last_checkpoint.right + L* last_checkpoint.forward_vector) (last_checkpoint.right - L* last_checkpoint.forward_vector)];
                %         tangent_obst = [(closest_obst_point + L * [0 -1; 1 0] * normal_vector) (closest_obst_point - L * [0 -1; 1 0] * normal_vector)];
                %         
                %         plot(tangent_left(1,:), tangent_left(2,:),'k--','LineWidth',1)
                %         plot(tangent_right(1,:), tangent_right(2,:),'k--','LineWidth',1)
                %         plot(tangent_obst(1,:), tangent_obst(2,:),'k--','LineWidth',1)
                %         
                %     elseif strcmp(p.name, 'SCR')
                %         import utils.con2vert
                %         poly_idx = controller_output.track_polygon_indices(end);
                %         poly = p.track_polygons(poly_idx);
                %         vertices = con2vert(poly.A, poly.b);
                %         K = convhull(vertices(:,1), vertices(:,2));
                %         vertices = vertices(K,:);
                %         plot(vertices(:,1), vertices(:,2),'k','LineWidth',1.5);
                %       OLD% 
                %         %     for i = 1:length(track.polytopes)
                %         %     %for i = ((1:3)+16)
                %         %         vertices = track.vertices(:,track.polytopes(i).vertex_indices);
                %         %         [K,V]=convhull(vertices');
                %         %         vertices = vertices(:,K(:,1));
                %         %         plot(vertices(1,:), vertices(2,:),':','LineWidth',3);
                %         %     end
                %     end

                %% Obstacle constraints for current position if obstacle has to be respected
                if sum(ws.obstacleTable(k,:)) >= 1
                    % iterate over all opponents
                    for j = 1:length(scn.vhs)
                        % Respect only constraints for opponents marked as obstacles
                        if ws.obstacleTable(k,j) == 1
                            d = pdist([x_0_controller(1:2)';ws.vhs{1,j}.x_0(1:2)'],'euclidean'); % calculate distance
                            normal_vector = - (x_0_controller(1:2)-ws.vhs{1,j}.x_0(1:2))/d; % normal vector in direction from trajectory point to obstacle center
                            closest_obst_point = ws.vhs{1,j}.x_0(1:2) - normal_vector * scn.vhs{1,j}.distSafe2CenterVal; % intersection of safe radius and connection between trajectory point and obstacle center 
                            tangent_obst = [(closest_obst_point + L * [0 -1; 1 0] * normal_vector) (closest_obst_point - L * [0 -1; 1 0] * normal_vector)];
                            obj.add_tmp_handle(plot(tangent_obst(1,:), tangent_obst(2,:),'--','color',c(k, :),'LineWidth',1));
                        end
                    end
                end
            end
        end
    end
    
    methods (Access = private, Static)
        function plot_track(track_checkpoints, vehicle_width)
            left_points = [track_checkpoints.left];
            right_points = [track_checkpoints.right];
            forward_vectors = [track_checkpoints.forward_vector];

            % Draw track area
            pts=[fliplr(right_points) right_points(:,end) left_points left_points(:,1)];
            fill(pts(1,:), pts(2,:),[1 1 1]*.8,'EdgeAlpha',0)

            %% Draw track outline with extra width for the vehicle
            width = vehicle_width / 2;
            normals = width*[0 -1;1 0]*forward_vectors;
            left_points = left_points + normals;
            right_points = right_points - normals;

            if true
                plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)],'k','LineWidth',1);
                plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)],'k','LineWidth',1);
            else             
                % Use these commands to show single checkpoints
                plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)],'.k');
                plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)],'.k');
            end

            % Center track in figure, add padding (else will move due to
            % vehicle plotting
            bounds = [min([left_points right_points]');max([left_points right_points]')];    
            xlim(bounds(:,1))
            ylim(bounds(:,2))
            set(gca, 'Position', [0 0 1 1])
            xlim(mean(xlim) + diff(xlim) * [-1 1] * 0.6)
            ylim(mean(ylim) + diff(ylim) * [-1 1] * 0.6)    
            daspect([1 1 1])
            axis off
            xlabel('x [m]'); ylabel('y [m]')
            
            % add scale
            text(0.5, .6, 'Scale: 1m', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
            plot([0 1], [.5 .5], 'k', 'LineWidth', 1);
        end
    end
end

