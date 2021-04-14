function checkpoints = testTrack3
    
    % Basic oval track corresponding to new vehicle dimensions (length
    % 0.075 m, width 0.045 m).

    trackWidth = 0.2;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;

    checkpoints = track.add_turn(checkpoints, 0, 3, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 0.19, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 0.19, trackWidth);
    checkpoints = track.add_turn(checkpoints, 0, 3, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 0.19, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 0.19, trackWidth);
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
    
end
