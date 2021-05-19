function track = main(checkpoints, epsilon_area_tolerance)
    % generate restricted track subset
    
    % 1) tesselation (based on discret track checkpoints)
    track = controller.SCR.generate_track_polygons.tesselate(checkpoints);
    % 2) merge polygons
    track = controller.SCR.generate_track_polygons.merge_polygons(track, epsilon_area_tolerance);
    % 3) add overlaps
    track = controller.SCR.generate_track_polygons.add_overlaps(track);
    %% calculate forward directions of polygons
    track = controller.SCR.generate_track_polygons.add_forward_directions(track, checkpoints);
end