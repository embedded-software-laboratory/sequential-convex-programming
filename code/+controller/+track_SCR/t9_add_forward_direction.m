function track = t9_add_forward_direction(track, checkpoints)
% average the length norm forward vector (mean of x and y)? for every
% polygon
n_checkpoints = length(checkpoints);
forward_vectors = [checkpoints.forward_vector];
for i = 1:length(track.polygons)
    A = track.polygons(i).A;
    b = track.polygons(i).b;

    % get all checkpoints in current polygon
    included_checkpoints_selector = ...
        all(A * [checkpoints.center] < repmat(b, 1, n_checkpoints));
    
    % average forward vectors of included checkpoints
    track.polygons(i).forward_direction = ...
        mean(forward_vectors(:, included_checkpoints_selector), 2);
end