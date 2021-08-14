function [track_fullSCR, track_tesselated, track_merged] = main(checkpoints, track_scale, epsilon_area_tolerance)
    % generate restricted track subset
    
    %% Actual SCR
    % 1) tesselation (based on discret track checkpoints)
    track_tesselated = controller.track_SCR.t1_tesselate(checkpoints);
    % 2) merge polygons
    track_merged = controller.track_SCR.t2_merge(track_tesselated, track_scale, epsilon_area_tolerance);
    % 3) add overlaps (first forwards, than backwards [herbey unioning forward an backward overlap])
    track_overlapped_forward = controller.track_SCR.t3_overlap(checkpoints, track_merged, track_scale, true);
    track_overlapped = controller.track_SCR.t3_overlap(checkpoints, track_merged, track_scale, false, track_overlapped_forward);
    
    % calculate forward directions of polygons
    track_fullSCR = controller.track_SCR.t9_add_forward_direction(track_overlapped, checkpoints);
    
    
    %% enlarge track_merged's constraints for polygon search (/ extract centroids)
    constraints_upscaled = struct;
    %centroids = nan(2, length(track_fullSCR.polygons));
    for j = 1:length(track_fullSCR.polygons)
        % enlarge polygon slighlty for numerical reasons
        poly_current = utils.poly.vert2poly(utils.poly.get_track_polygon_vertices(j, track_fullSCR));
        [poly_current_center(1), poly_current_center(2)] = poly_current.centroid;
        % scale down w.r.t. center
        %   scaling down enlarged so that only track width is
        %   reduced minimallistically (as enlarged and next
        %   neighbour overlap --> scaling has no effect here)
        poly_current = poly_current.scale(1 + 1e-3, poly_current_center);
        [constraints_upscaled(j).A, constraints_upscaled(j).b] = utils.poly.vert2con(utils.poly.poly2vert(poly_current)');
        
        %[centroids(1, j), centroids(2, j)] = poly_current.centroid();
    end
    % save
    track_fullSCR.constraints_upscaled = constraints_upscaled;
    %track_fullSCR.centroids = centroids;
end