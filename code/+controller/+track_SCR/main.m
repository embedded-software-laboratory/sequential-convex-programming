function track = main(checkpoints, track_scale, epsilon_area_tolerance)
    % generate restricted track subset
    
    % 1) tesselation (based on discret track checkpoints)
    track = controller.track_SCR.t1_tesselate(checkpoints);
    % 2) merge polygons
    track = controller.track_SCR.t2_merge(track, track_scale, epsilon_area_tolerance);
    % 3) add overlaps (first forwards, than backwards)
    track_overlapped_forward = controller.track_SCR.t3_overlap(track, track_scale, true);
    track = controller.track_SCR.t3_overlap(track, track_scale, false, track_overlapped_forward);
    
    % calculate forward directions of polygons
    track = controller.track_SCR.t9_add_forward_direction(track, checkpoints);
end