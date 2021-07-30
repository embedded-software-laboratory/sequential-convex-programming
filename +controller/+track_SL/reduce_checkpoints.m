function checkpoints = reduce_checkpoints(checkpoints, distance_min, track_creation_scale)
% reduces numbber of checkpoints to have minimum given distance

if distance_min > 0
    n_checkpoints_original = length(checkpoints);

    % iterate in reverse: deleting polygons in the loop
    for i = length(checkpoints):-1:2
        [~, distance_cps] = controller.track_SL.find_closest_checkpoint_index(checkpoints(i).center, checkpoints(i - 1).center);
        
        % if distance to last checkpoint is smaller than threshold
        if distance_cps < distance_min * track_creation_scale
            % delete current checkpoint
            checkpoints(i - 1) = [];
        end
    end
    
    fprintf('\tReduced number of track checkpoints\n\t\t from %i to %i (reduction to %i%%)\n',...
        n_checkpoints_original, length(checkpoints), round(length(checkpoints)/n_checkpoints_original*100));
end