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
        poly_current = vert2poly(get_track_polygon_vertices(j, track));
        [poly_current_center(1), poly_current_center(2)] = poly_current.centroid;
        % scale down w.r.t. center
        %   scaling down enlarged so that only track width is
        %   reduced minimallistically (as enlarged and next
        %   neighbour overlap --> scaling has no effect here)
        poly_current = poly_current.scale(1 + 1e-3, poly_current_center);
        [constraints_upscaled(j).A, constraints_upscaled(j).b] = utils.vert2con(poly2vert(poly_current)');
    end
    % save
    track.constraints_upscaled = constraints_upscaled;
end

    
function p_poly = vert2poly(p_vert)
    % convert from vertice representation to MATLAB's polyshape
    
    % convert to correctly ordered polygon (current vertices are unordered)
    p_vert = cleanse_convex_polygon(p_vert);
    %if debug_; plot_polygon(p_vert); end
    % ... so that we can convert to MATLAB's poly
    p_poly = polyshape(p_vert(:, 1), p_vert(:, 2));
    %if debug_; plot(p_poly); end
end


function p_vert = poly2vert(p_poly)
    p_vert = p_poly.Vertices';
end

function p_vert = get_track_polygon_vertices(i, track)
    p_vert = track.vertices(:,track.polygons(i).vertex_indices)';
end

function polygon = cleanse_convex_polygon(polygon)
    % order vertices, remove unneccesary vertices
    select_and_reorder_indices = convhull(polygon);
    select_and_reorder_indices = select_and_reorder_indices(1:end-1);
    polygon = polygon(select_and_reorder_indices, :);
end