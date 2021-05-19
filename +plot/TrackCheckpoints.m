classdef TrackCheckpoints < plot.Base
    % plot discretized track's checkpoints
    %
    % usage example `plot.TrackCheckpoints().plot(model.track.Hockenheim4)`
    
    methods
        function plot(obj, checkpoints)
            % in case is distinct plot (need to check object as `isnan`
            % doesn't work with objects)
            if isobject(obj.figure_handle) || ~isnan(obj.figure_handle)
                set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
                set(obj.figure_handle, 'Name', 'Track - Discretized & Checkpoints');
            end
            
            hold on
            n = length(checkpoints);
            center = [checkpoints.center];
            left = [checkpoints.left];
            right = [checkpoints.right];

            % center line points
            scatter(center(1, :), center(2, :), 'm', '.')

            % normal vector
            plot(...
                reshape([left(1,:);right(1,:);nan(1,n)],3*n,1),...
                reshape([left(2,:);right(2,:);nan(1,n)],3*n,1),...
                'Color',[1 1 1]*0.7);

            % right points
            plot([left(1,:) left(1,1)],[left(2,:) left(2,1)], 'b');

            % left points
            plot([right(1,:) right(1,1)],[right(2,:) right(2,1)], 'r');

            
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