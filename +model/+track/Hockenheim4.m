function checkpoints = Hockenheim4

    % Compared to Hockenheim3: forwarded start-/finish-line by 0.5

    % The layout of the Grand Prix Circuit of Hockenheim, Germany, has been
    % used as an inspiration for this layout. The track is designed for
    % races of model scale cars of scale 1:43. However, the track layout
    % has a scale of about 1:125. This makes it more difficult for the cars
    % to maneuver.

    trackWidth = 0.25;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;
    
    % section 1
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 4.4, trackWidth); % 3.9834
    checkpoints = model.track.add_turn_corner(checkpoints, -75/360, 75/360 * 0.9702 * pi, trackWidth); % D = 0.9702; alpha = 90
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 2.01, trackWidth); % 1.8962
    
    % section 2
    checkpoints = model.track.add_turn_corner(checkpoints, -105/360, 105/360 * 0.5958 * pi, trackWidth); % D = 0.5958; alpha = 105
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.2943, trackWidth); % 0.2943
    checkpoints = model.track.add_turn_corner(checkpoints, -30/360, 30/360 * 0.5958 * pi, trackWidth); % D = 0.5958; alpha = 30
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.6381, trackWidth); % 0.6381
    checkpoints = model.track.add_turn_corner(checkpoints, 45/360, 45/360 * 1.3228 * pi, trackWidth); % D = 1.3228; alpha = 30
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 1.9571, trackWidth); % 1.9571
    checkpoints = model.track.add_turn_corner(checkpoints, 45/360, 45/360 * 2.2246 * pi, trackWidth); % D = 2.2246; alpha = 45
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.5332, trackWidth); % 0.5332
    checkpoints = model.track.add_turn_corner(checkpoints, -30/360, 30/360 * 1.5899 * pi, trackWidth); % D = 1.5899; alpha = 30
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.2704, trackWidth); % 0.2704
    checkpoints = model.track.add_turn_corner(checkpoints, -105/360, 105/360 * 0.7923 * pi, trackWidth); % D = 0.7923; alpha = 105
    
    % section 3
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.8, trackWidth); % 0.4577
    checkpoints = model.track.add_turn_corner(checkpoints, -90/360, 90/360 * 1.4027 * pi, trackWidth); % D = 1.4027; alpha = 75
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 1.7915, trackWidth); % 1.7915
    checkpoints = model.track.add_turn_corner(checkpoints, 135/360, 135/360 * 0.6866 * pi, trackWidth); % D = 0.6866; alpha = 135
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.6, trackWidth); % 0.7061
    checkpoints = model.track.add_turn_corner(checkpoints, 75/360, 75/360 * 1 * pi, trackWidth); % D = 0.7065; alpha = 75
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.2146, trackWidth); % 0.2146
    checkpoints = model.track.add_turn_corner(checkpoints, -30/360, 30/360 * 0.8 * pi, trackWidth); % D = 0.7178; alpha = 45
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.5829, trackWidth); % 0.5829
    checkpoints = model.track.add_turn_corner(checkpoints, -75/360, 75/360 * 0.7 * pi, trackWidth); % D = 0.4948; alpha = 75
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.5439, trackWidth); % 0.5439
    checkpoints = model.track.add_turn_corner(checkpoints, -120/360, 105/360 * 1.1826 * pi, trackWidth); % D = 1.1826; alpha = 120
    
    checkpoints = model.track.add_turn_straight(checkpoints, 0, 0.5, trackWidth);
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
    
end

