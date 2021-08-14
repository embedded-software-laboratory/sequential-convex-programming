function track_new = t3_overlap(checkpoints, track, track_scale, direction_is_forward, track_overlapped)
% Inputs
% checkpoints: just for plotting
% track
% track_overlapped: in case one direction was run already, contains
%           overlapped track for merging of forward and backward overlaps.
%           if first run, should equal arg `track`
% track_scale [m]

% convert to constraints
for i = 1:length(track.polygons)
    [track.polygons(i).A, track.polygons(i).b] = utils.poly.vert2con(track.vertices(:,track.polygons(i).vertex_indices)');
end

track_new = struct;
track_new.vertices = nan(2,0);


debug_ = false;
if debug_
    figure(997) %#ok<*UNRCH> % for whole file ok
    clf
    hold on
    plots.Race.plot_track(checkpoints);
end

% find neighbor intersections
for i_p = 1:length(track.polygons)
    %% find shared vertices of directly neighboured polygons
    % leveraging track definition: each polygon has exactly two neighbours
    % with exactly on full shared edge each (equaling exactly two vertices
    % each)
    %
    % NOTE: following sections (`%%`) will reference documentation from
    % file '/algorithm_expand_convex_polygon.pdf'
    %
    % Subscripts
    %   p: current polygon to check
    %   f: next forward polygon neighbour
    %   b: next backward polygon neighbour
    
    %% (0) get current polygon
    p = utils.poly.vert2poly(utils.poly.get_track_polygon_vertices(i_p, track));
    if debug_; plot(p); end
    %% (1) enlarge current polygon
    p_enlarged = utils.poly.vert2poly(utils.poly.enlarge_polygon_into_forward_direction(i_p, track, track_scale, direction_is_forward, debug_));
    if debug_; plot(p_enlarged); end
    
    %% (2) get next polygon index
    if direction_is_forward
        i_f = utils.mod1(i_p + 1, length(track.polygons));
    else
        i_f = utils.mod1(i_p - 1, length(track.polygons));
    end
    %% (3) repeat as long as forward neighbours lay in current polygon's enlarged region
    while true
        if debug_
            figure(997)
            clf
            hold on
            plots.Race.plot_track(checkpoints);
            plot(p_enlarged)
        end

        %% (3a) get next neighbour polygon
        p_next_neighbour = utils.poly.vert2poly(utils.poly.get_track_polygon_vertices(i_f, track));
        if debug_; plot(p_next_neighbour); end
        
        %% (3b) if next forward neighbour overlaps enlarged current polygon
        if p_enlarged.overlaps(p_next_neighbour)
            %% (3b1) enlarge next forward neighbour
            p_next_neighbour_enlarged = utils.poly.vert2poly(utils.poly.enlarge_polygon_into_forward_direction(i_f, track, track_scale, direction_is_forward, debug_));
            if debug_; plot(p_next_neighbour_enlarged); end
            
            %% (3b2) get overlap between current enlarged polygon and enlarged
            % neighbour
            p_overlap_enlarged_n_next_neighbour = p_enlarged.intersect(p_next_neighbour_enlarged);
            if debug_; plot(p_overlap_enlarged_n_next_neighbour); end
            
            %% (3b3) divide current enlarged polygon by overlap with next
            % neighbour
            %   yields a polygon with two separated regions:
            %   on before overlap (backwards), one after (forwards)
            % get p_enlarged's center point
            [p_overlap_enlarged_n_next_neighbour_center(1), p_overlap_enlarged_n_next_neighbour_center(2)] = p_overlap_enlarged_n_next_neighbour.centroid;
            % scale down w.r.t. center
            %   scaling down enlarged so that only track width is
            %   reduced minimallistically (as enlarged and next
            %   neighbour overlap --> scaling has no effect here)
            p_overlap_enlarged_n_next_neighbour_upscaled = p_overlap_enlarged_n_next_neighbour.scale(1 + 1e-3, p_overlap_enlarged_n_next_neighbour_center);
            if debug_; plot(p_overlap_enlarged_n_next_neighbour_upscaled); end

            % re-try to divide enlarged area as before
            p_enlarged_divided_by_neighbour_overlap = p_enlarged.subtract(p_overlap_enlarged_n_next_neighbour_upscaled);
            if debug_; plot(p_enlarged_divided_by_neighbour_overlap); end
            %if debug_; scatter(p_enlarged_divided_by_neighbour_overlap.Vertices(:, 1), p_enlarged_divided_by_neighbour_overlap.Vertices(:, 2)); end

            if p_enlarged_divided_by_neighbour_overlap.NumRegions == 1
                % edge case: neighbour doesn't divide p_enlarged,
                % because it's only sticking into p_enlarged but not
                % laying inside
                p_enlarged_retain = p_enlarged_divided_by_neighbour_overlap;
            else
                % case 3 regions can happen soldomly, if neighbour
                % restricts track like a triangle but has same normal
                % as the current polygon
                %   --> especially at joint point (when track circle is
                %   close) when running in backward direction
                %
                % integrated special exclusion (so that we assert as
                % tight as possible, detecting possible errors as early
                % as possible)
                if p_enlarged_divided_by_neighbour_overlap.NumRegions == 3 && i_p == 1 && i_f == length(track.polygons)
                    warning('Ignoring set division issue at track joint (where circle is closed). Caused by slight misalignments from track creation')
                else
                    assert(p_enlarged_divided_by_neighbour_overlap.NumRegions == 2, 'Numerical issues? Please investigate!')
                end


                %% (3b4) check which polygon is part of original polygon (= overlaps)
                %   should be the first polygon due to track definition
                tf_p_part_of_original = overlaps([p; p_enlarged_divided_by_neighbour_overlap.regions]);

                % choose original overlapping polygon
                %   we want: polygon 2 and 3 (columns) relation to first polygon (row)
                p_divided_selection_index = tf_p_part_of_original(1, 2:3);
                temp = p_enlarged_divided_by_neighbour_overlap.regions;
                p_enlarged_retain = temp(p_divided_selection_index);
            end
            if debug_; plot(p_enlarged_retain); end
            
            %% (3b5) union: reatining old p_enlarged parts, add overlap with next neighbour
            [p_enlarged_retain_center(1), p_enlarged_retain_center(2)] = p_enlarged_retain.centroid;
            % scale down w.r.t. center
            %   scaling down enlarged so that only track width is
            %   reduced minimallistically (as enlarged and next
            %   neighbour overlap --> scaling has no effect here)
            p_enlarged_retain_upscaled = p_enlarged_retain.scale(1 + 1e-3, p_enlarged_retain_center);
                
            p_enlarged = union(p_enlarged_retain_upscaled, p_overlap_enlarged_n_next_neighbour_upscaled);
            
            % downscale again (else we exponentially increase polygon size
            % over loop iterations)
            
            [p_enlarged_center(1), p_enlarged_center(2)] = p_enlarged.centroid;
            % scale down w.r.t. center
            %   scaling down enlarged so that only track width is
            %   reduced minimallistically (as enlarged and next
            %   neighbour overlap --> scaling has no effect here)
            p_enlarged = p_enlarged.scale(1 - 1e-3, p_enlarged_center);
            
            if debug_; plot(p_enlarged, 'EdgeColor', 'r'); end
            %if debug_; drawnow; end
        %% (3c)
        else
            %% (3c1) no overlapping --> maximum convex enlargement reached
            break
        %% (3d)
        end
        
        %% (3e) for-loop: forward index
        if direction_is_forward
            i_f = utils.mod1(i_f + 1, length(track.polygons));
        else
            i_f = utils.mod1(i_f - 1, length(track.polygons));
        end
    end
    if debug_; plot(p_enlarged, 'EdgeColor', 'r'); end
    %if debug_; drawnow; pause(0.2); end
    
    %% save output
    track_new.polygons(i_p).vertex_indices = length(track_new.vertices) + (1:length(utils.poly.poly2vert(p_enlarged)));
    track_new.vertices = [track_new.vertices  utils.poly.poly2vert(p_enlarged)];
    [track_new.polygons(i_p).A, track_new.polygons(i_p).b] = utils.poly.vert2con(utils.poly.poly2vert(p_enlarged)');   
end

%% merge forward and backward overlap, if previous overlaps given
if exist('track_overlapped', 'var')
    track_merged_overlaps = struct;
    track_merged_overlaps.vertices = nan(2,0);
    
    for i_p = 1:length(track.polygons)
        if debug_
            figure(997)
            clf
            hold on
            plots.Race.plot_track(checkpoints);
        end
        
        % get overlaps in both directions
        p_overlap_prev = utils.poly.vert2poly(utils.poly.get_track_polygon_vertices(i_p, track_overlapped));
        p_overlap_curr = utils.poly.vert2poly(utils.poly.get_track_polygon_vertices(i_p, track_new));
        if debug_; plot(p_overlap_prev); plot(p_overlap_curr); end
        
        % merge overlaps
        p_overlap = p_overlap_prev.union(p_overlap_curr);
        if debug_; plot(p_overlap, 'EdgeColor', 'r'); end
        
        %% save output
        track_merged_overlaps.polygons(i_p).vertex_indices = length(track_merged_overlaps.vertices) + (1:length(utils.poly.poly2vert(p_overlap)));
        track_merged_overlaps.vertices = [track_merged_overlaps.vertices  utils.poly.poly2vert(p_overlap)];
        [track_merged_overlaps.polygons(i_p).A, track_merged_overlaps.polygons(i_p).b] = utils.poly.vert2con(utils.poly.poly2vert(p_overlap)');   
    end
    
    track_new = track_merged_overlaps;
end
end
