function checkpoints = testTrack1
% scale 1:1
% by Maczijewski
trackWidth = 7;

checkpoints = struct; % contains all relevatn information/a definition of the track layout

checkpoints.left = [0; trackWidth/2];
checkpoints.right = [0; -trackWidth/2];
checkpoints.center = [0; 0]; % row vector with x and y coordinate
checkpoints.yaw = 0;
checkpoints.forward_vector = [1; 0];

checkpoints = model.track.add_turn(checkpoints, 0, 76, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.25, 50, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.25, 8, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.1, 30, trackWidth);
checkpoints = model.track.add_turn(checkpoints, 0.1, 30, trackWidth);
checkpoints = model.track.add_turn(checkpoints, 0.5, 15, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.5, 30, trackWidth);
checkpoints = model.track.add_turn(checkpoints, 0.5, 15, trackWidth);
checkpoints = model.track.add_turn(checkpoints, 0, 60, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.25, 10, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.25, 20, trackWidth);
checkpoints = model.track.add_turn(checkpoints, 0, 55, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.25, 90.3, trackWidth);
checkpoints = model.track.add_turn(checkpoints, 0, 5.3, trackWidth);
checkpoints = model.track.add_turn(checkpoints, -0.25, 20, trackWidth);

checkpoints = checkpoints(2:end); % select checkpoints 2 till end
end
