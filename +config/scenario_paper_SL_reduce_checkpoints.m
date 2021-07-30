function cfg = scenario_paper_SL_reduce_checkpoints(cfg, min_distance)
% reduce number of checkpoints of track discretization

for i = 2:length(checkpoints)
    % if distance to last checkpoint is smaller than threshold
    if (checkpoints(i) - checkpoints(i-1)> min_distance
        % delete current checkpoint
        del checkpoints(i)
        % reset index to match current selection
        i = i - 1;
    end
end