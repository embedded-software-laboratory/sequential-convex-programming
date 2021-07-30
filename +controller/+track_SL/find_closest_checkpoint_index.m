function cp_indices = find_closest_checkpoint_index(pos, cp_center, Hp)
% find closest checkpoint(s) of track to given state(s) (position) x
% Inputs:
%   cp:     track or subset of track
%   pos:    position of (x, y)
%   Hp:     number of checkpoint to check, starting from x(1). Supply 1 if
%           only one checkpoint/state pair is of interest

cp_indices = nan(1, Hp);
for k = 1:Hp
    % expand position of state for comparison with checkpoints
    x_position_compare = repmat(pos(:, k), 1, length(cp_center));
    % take euclidian distance (for x and y) for each center-checkpoint
    euclidian_distance = sum((cp_center - x_position_compare).^2);
    % save index of closest checkpoint
    [~, cp_indices(k)] = min(euclidian_distance);
end
end