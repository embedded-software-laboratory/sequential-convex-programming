function checkpoints = testTrack2

    % Oval track corresponding to original dimensions of the vehicles
    % according to Maczijewski 2017.

    trackWidth = 10;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;

    checkpoints = track.add_turn(checkpoints, 0, 80, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 50, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 50, trackWidth);
    checkpoints = track.add_turn(checkpoints, 0, 80, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 50, trackWidth);
    checkpoints = track.add_turn(checkpoints, -0.25, 50, trackWidth);
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
    
end
