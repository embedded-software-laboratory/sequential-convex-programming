function track = main(checkpoints, epsilon_area_tolerance)
    % generate restricted track subset
    
    %% 1) tesselation (based on discret track checkpoints)
    %%
    track = struct;
    
    % Interlace left and right track boundary points
    track.vertices = nan(2, 2*length(checkpoints));
    track.vertices(:,1:2:end) = [checkpoints.left];
    track.vertices(:,2:2:end) = [checkpoints.right];

    % Generate initial set of polygons (quadrilaterals) that cover the track
    for i = 1:length(checkpoints)
        track.polygons(i).vertex_indices = utils.mod1(2*i + (-1:2), size(track.vertices, 2));
    end

    %% 2) merge polygons
    track = controller.SCR.generate_track_polygons.merge_polygons(track, epsilon_area_tolerance);
    %% 3) add overlaps
    track = controller.SCR.generate_track_polygons.add_overlaps(track);
    %% calculate forward directions of polygons
    track = controller.SCR.generate_track_polygons.add_forward_directions(track, checkpoints);
end