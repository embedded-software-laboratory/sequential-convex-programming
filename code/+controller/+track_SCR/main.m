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
    
    
    % enlarge constraints for polygon search
    constraints_upscaled = struct;
    for j = 1:length(track.polygons)
        % enlarge polygon slighlty for numerical reasons
        poly_current = utils.poly.vert2poly(utils.poly.get_track_polygon_vertices(j, track));
        [poly_current_center(1), poly_current_center(2)] = poly_current.centroid;
        % scale down w.r.t. center
        %   scaling down enlarged so that only track width is
        %   reduced minimallistically (as enlarged and next
        %   neighbour overlap --> scaling has no effect here)
        poly_current = poly_current.scale(1 + 1e-3, poly_current_center);
        [constraints_upscaled(j).A, constraints_upscaled(j).b] = utils.poly.vert2con(utils.poly.poly2vert(poly_current)');
    end
    % save
    track.constraints_upscaled = constraints_upscaled;
end