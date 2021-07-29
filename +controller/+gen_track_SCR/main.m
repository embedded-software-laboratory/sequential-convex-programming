function track = main(checkpoints, track_scale, epsilon_area_tolerance)
    % generate restricted track subset
    
    % 1) tesselation (based on discret track checkpoints)
    track = controller.gen_track_SCR.t1_tesselate(checkpoints);
    % 2) merge polygons
    track = controller.gen_track_SCR.t2_merge(track, track_scale, epsilon_area_tolerance);
    % 3) add overlaps
    track = controller.gen_track_SCR.t3_overlap(track, track_scale);
    
    % calculate forward directions of polygons
    track = controller.gen_track_SCR.t9_add_forward_direction(track, checkpoints);
end