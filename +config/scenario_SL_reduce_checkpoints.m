function cfg = scenario_SL_reduce_checkpoints(cfg)
% reduce number of checkpoints of track discretization

% will be scaled arroding to track scale
%   equals 1m on 1:1 scale, ~0.023m on 1:43 scale
cfg.scn.track_checkpoints_distance_min = 0.023 * 43/1;