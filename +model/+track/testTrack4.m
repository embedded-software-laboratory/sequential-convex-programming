function checkpoints = testTrack4
    
    % Oval track corresponding to new vehicle dimensions (length
    % 0.075 m, width 0.045 m).
    
    % Adapted number of checkpoints (more on straights, less in corners).

    trackWidth = 0.2;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;

    checkpoints = model.track.add_turn_straight(checkpoints, 0, 3, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.5, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.5, trackWidth);
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 3, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.5, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.5, trackWidth);
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
    
end

