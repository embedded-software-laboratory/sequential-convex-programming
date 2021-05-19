track_checkpoints = model.track.Hockenheim4;
close all

plotCheckpoints(track_checkpoints)

function plotCheckpoints(checkpoints)
    figure('Name', 'Track - Discretized & Checkpoints')
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
    
    legend(...
        'Center Line Points',...
        'Normal Vectors',...
        'Right Track Points (Connected)',...
        'Left Track Points (Connected)')
end
