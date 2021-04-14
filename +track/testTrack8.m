function checkpoints = testTrack8
    
    % Real track layout similar to Liniger Papers 2014-2018
    
    % Adapted number of checkpoints (more on straights, less in corners).

    trackWidth = 0.3;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;

    checkpoints = track.add_turn_straight(checkpoints, 0, 1.25, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.25, 1.5, trackWidth);
    checkpoints = track.add_turn_straight(checkpoints, 0, 1, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.25, 0.8, trackWidth);
    
    checkpoints = track.add_turn_straight(checkpoints, 0, 0.4, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, 0.25, 0.4, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.25, 0.4, trackWidth);
    checkpoints = track.add_turn_straight(checkpoints, 0, 0.6, trackWidth);
    
    checkpoints = track.add_turn_corner(checkpoints, -0.5, 0.6, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, 0.5, 0.6, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.25, 0.3, trackWidth);
    checkpoints = track.add_turn_straight(checkpoints, 0, 0.4, trackWidth);
    
    checkpoints = track.add_turn_corner(checkpoints, -0.375, 0.525, trackWidth);
    checkpoints = track.add_turn_straight(checkpoints, 0, 0.6, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, 0.375, 1.3, trackWidth);
    checkpoints = track.add_turn_straight(checkpoints, 0, 0.6, trackWidth);
    
    checkpoints = track.add_turn_corner(checkpoints, 0.25, 0.6, trackWidth);
    checkpoints = track.add_turn_straight(checkpoints, 0, 0.3, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, 0.375, 0.525, trackWidth);
    checkpoints = track.add_turn_straight(checkpoints, 0, 0.5, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.5, 0.6, trackWidth);
    
    checkpoints = track.add_turn_straight(checkpoints, 0, 1.6, trackWidth);
    checkpoints = track.add_turn_corner(checkpoints, -0.375, 0.525, trackWidth);
    
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
    
end

