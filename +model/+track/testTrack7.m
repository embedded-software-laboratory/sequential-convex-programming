function checkpoints = testTrack7
    
    % Real track layout similar to Janis
    
    % Adapted number of checkpoints (more on straights, less in corners).

    trackWidth = 0.3;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;

    checkpoints = model.track.add_turn_straight(checkpoints, 0, 2.9, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 2, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.3, trackWidth);
    
    checkpoints = model.track.add_turn_corner(checkpoints, -0.1, 1.2, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, 0.1, 1.2, trackWidth);
    
    checkpoints = model.track.add_turn_corner(checkpoints, 0.5, 0.7, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.5, 1.2, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, 0.5, 0.7, trackWidth);
    
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 2.4, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.4, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.8, trackWidth);
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 2.2, trackWidth);
    
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 3.5, trackWidth);
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.39, trackWidth);
    checkpoints = model.track.add_turn_corner(checkpoints, -0.25, 0.8, trackWidth);
    
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
    
end

