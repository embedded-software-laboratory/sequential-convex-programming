function [checkpoints, creation_scale] = HockenheimGrandPrix
% The layout of the Grand Prix Circuit of Hockenheim, Germany, has been
% used as an inspiration for this layout. The track is designed for
% races of model scale cars of scale 1:43. However, the track layout
% has a scale of about 1:145. This makes it more difficult for the cars
% to maneuver.
% 
% FIXME which scale? trackWidth is the same as other HockenheimShort, but
% 	states other scale
creation_scale = 1/145;

trackWidth = 0.25;

checkpoints = struct;
checkpoints.left = [0; trackWidth/2];
checkpoints.right = [0; -trackWidth/2];
checkpoints.center = [0; 0];
checkpoints.yaw = 0;
checkpoints.forward_vector = [1; 0];
checkpoints.ds = 0;

% section 1
checkpoints = model.track.add_turn_N50(checkpoints, 0, 5.6, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, -5/24, 5/24 * 0.96 * pi, trackWidth); % D=24 -> 0.96, W=75° -> 5/24 %9%
checkpoints = model.track.add_turn_N50(checkpoints, 0, 4.2, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, -7/24, 7/24 * 0.48 * pi, trackWidth); % D=12 -> 0.48, W=105° -> 7/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.6, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, 3/24, 3/24 * 0.96 * pi, trackWidth); % D=24 -> 0.96, W=60° -> 4/24 %5%

% section 2
checkpoints = model.track.add_turn_N50(checkpoints, 0, 1.8, trackWidth); % done %13%
checkpoints = model.track.add_turn_N30(checkpoints, 2/24, 2/24 * 12 * pi, trackWidth); % D=293 -> 11.72, W=45° -> 3/24 %3% %8% %13%
checkpoints = model.track.add_turn_N50(checkpoints, 0, 2.5, trackWidth); % done %7%
checkpoints = model.track.add_turn_N30(checkpoints, -10/24, 10/24 * 0.5 * pi, trackWidth); % D=7 -> 0.28, W=150° -> 10/24 %1%
checkpoints = model.track.add_turn_N50(checkpoints, 0, 2.6, trackWidth); % done %10%
checkpoints = model.track.add_turn_N30(checkpoints, -4/24, 4/24 * 0.52 * pi, trackWidth); % D=13 -> 0.52, W=60° -> 4/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.4, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, 2/24, 2/24 * 0.56 * pi, trackWidth); % D=14 -> 0.56, W=30° -> 2/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 1.5, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, 4/24, 4/24 * 0.35 * pi, trackWidth); % D=7 -> 0.28, W=75° -> 5/24 %2%
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.4, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, 3/24, 3/24 * 0.84 * pi, trackWidth); % D=21 -> 0.84, W=45° -> 3/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.5, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, -4/24, 4/24 * 0.76 * pi, trackWidth); % D=19 -> 0.76, W=60° -> 4/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.4, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, -1/24, 4/24 * 1.48 * pi, trackWidth); % D=37 -> 1.48, W=15° -> 1/24

% section 3
checkpoints = model.track.add_turn_N50(checkpoints, 0, 1.6, trackWidth); % done %6% %10%
checkpoints = model.track.add_turn_N30(checkpoints, -6/24, 6/24 * 1.4 * pi, trackWidth); % D=35 -> 1.4, W=90° -> 6/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 1.4, trackWidth); % done %12%
checkpoints = model.track.add_turn_N30(checkpoints, 9/24, 9/24 * 0.68 * pi, trackWidth); % D=17 -> 0.68, W=135° -> 9/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.7, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, 4/24, 4/24 * 0.72 * pi, trackWidth); % D=18 -> 0.72, W=60° -> 4/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.2, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, -2/24, 2/24 * 0.72 * pi, trackWidth); % D=18 -> 0.72, W=45° -> 3/24 %4% %11%
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.8, trackWidth); % done %11%
checkpoints = model.track.add_turn_N30(checkpoints, -5/24, 5/24 * 0.48 * pi, trackWidth); % D=12 -> 0.48, W=75° -> 5/24
checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.3, trackWidth); % done
checkpoints = model.track.add_turn_N30(checkpoints, -7/24, 7/24 * 1.2 * pi, trackWidth); % D=30 -> 1.2, W=120° -> 8/24

checkpoints = checkpoints(2:end); % select checkpoints 2 till end  
end