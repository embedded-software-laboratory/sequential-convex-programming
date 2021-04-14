function checkpoints = testTrack6
    
    % Circular track corresponding to new vehicle dimensions (length
    % 0.075 m, width 0.045 m) for provoked oversteering/drifting.
    
    % Adapted number of checkpoints (more on straights, less in corners).

    trackWidth = 0.3;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;

    checkpoints = track.add_turn_corner(checkpoints, -0.25, 0.25, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.25, 0.25, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.25, 0.25, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.25, 0.25, trackWidth);
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
    
end
