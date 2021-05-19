function [checkpoints, creation_scale] = testCircuitSpline(n_spline)
% n_spline e.g. 10
% was called TestTrack1
creation_scale = 1/1;

if ~exist('n_spline', 'var'); n_spline = 10; end

spline = struct;
spline(1).point = [58 106]';
spline(1).tangent = [-10 -27]';

spline(2).point = [52 45]';
spline(2).tangent = [10 -30]';

spline(3).point = [50 8]';
spline(3).tangent = [-15 -5]';

spline(4).point = [22 10]';
spline(4).tangent = [-15 5]';

spline(5).point = [17 48]';
spline(5).tangent = [0 15]';

spline(6).point = [14 62]';
spline(6).tangent = [-4 2]';

spline(7).point = [3 63]';
spline(7).tangent = 1.2*[-4 3]';

spline(8).point = [11 81]';
spline(8).tangent = [12 10]';

spline(9).point = [32 115]';
spline(9).tangent = [6 20]';

spline(10).point = [52 133]';
spline(10).tangent = [15 -5]';

spline(11).point = [58 106]';
spline(11).tangent = [-10 -27]';

for i=1:length(spline)
    spline(i).point = spline(i).point + [-39 -6]';
end

trackWidth = 4.5;
checkpoints = model.track.spline_to_checkpoints(spline, trackWidth, n_spline);

% remove double entry (start & end)
checkpoints = checkpoints(2:end);
end