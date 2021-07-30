classdef TrackCheckpoints < plot.Base
    % plot discretized track's checkpoints
    %
    % usage example
    % ```
    % [checkpoints, track_creation_scale] = model.track.HockenheimShort()
    % plot.TrackCheckpoints().plot(checkpoints, track_creation_scale)
    % ```
    
    methods
        function plot(obj, checkpoints, track_creation_scale)
            % in case is distinct plot (need to check object as `isnan`
            % doesn't work with objects)
            if isobject(obj.figure_handle) || ~isnan(obj.figure_handle)
                set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
                set(obj.figure_handle, 'Name', sprintf('Track - Discretized & Checkpoints (scale %s)', utils.rat2str(track_creation_scale)));
            end
            
            hold on
            n = length(checkpoints);
            center_points = [checkpoints.center];
            left_points = [checkpoints.left];
            right_points = [checkpoints.right];

            % center line points
            scatter(center_points(1, :), center_points(2, :), 'm', '.')

            % normal vector
            plot(...
                reshape([left_points(1,:);right_points(1,:);nan(1,n)],3*n,1),...
                reshape([left_points(2,:);right_points(2,:);nan(1,n)],3*n,1),...
                'Color',[1 1 1]*0.7);

            % left points
            plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)], 'b');

            % right points
            plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)], 'r');

            
            if isobject(obj.figure_handle) || ~isnan(obj.figure_handle)
                xlabel('x [m]'); ylabel('y [m]')
            end
            legend(...
                'Center Line Points',...
                'Normal Vectors',...
                'Right Track Points (Connected)',...
                'Left Track Points (Connected)')
            axis equal
        end
    end
end