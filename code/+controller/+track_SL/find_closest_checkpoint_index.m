function [cp_index, distance] = find_closest_checkpoint_index(position, cp_center)
% find closest checkpoint of track to given state (position) x
% Inputs:
%   cp:     track or subset of track
%   pos:    position of (x, y)

% expand positions to match size of checkpoints
position_expanded = repmat(position, 1, length(cp_center));

% take euclidian distance (for x and y) for each center-checkpoint
euclidian_distance = sum((cp_center - position_expanded).^2);

% save index of closest checkpoint
[distance, cp_index] = min(euclidian_distance);