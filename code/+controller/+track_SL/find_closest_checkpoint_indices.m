function cp_indices = find_closest_checkpoint_indices(positions, cp_center, Hp)
% find closest checkpoints of track to given states (position) x
% Inputs:
%   cp:     track or subset of track
%   pos:    position of (x, y)
%   Hp:     number of checkpoint to check, starting from x(1). Supply 1 if
%           only one checkpoint/state pair is of interest

% preallocate for speed
cp_indices = nan(1, Hp);

for k = 1:Hp
    % save index of closest checkpoint
    [cp_indices(k), ~] = controller.track_SL.find_closest_checkpoint_index(positions(:, k), cp_center);
end