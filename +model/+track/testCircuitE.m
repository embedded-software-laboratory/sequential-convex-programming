function [checkpoints, creation_scale] = testCircuitE
% scale 1:1
% by Maczijewski
% was called "hockenheim_simple" despite having not similarity
% creating simple, artificial hockenheim track
creation_scale = 1/1;

trackWidth = 7;

% start point
checkpoints = struct;
checkpoints.left = [0; trackWidth/2];
checkpoints.right = [0; -trackWidth/2];
checkpoints.center = [0; 0];
checkpoints.yaw = 0;
checkpoints.forward_vector = [1; 0];

% construct track
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

% remove double entry (start & end)
checkpoints = checkpoints(2:end);
end