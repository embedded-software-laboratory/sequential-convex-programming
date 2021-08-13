function track_new = t3_overlap(track, track_scale, direction_is_forward, track_overlapped)
% Inputs
% track
% track_overlapped: in case one direction was run already, contains
%           overlapped track for merging of forward and backward overlaps.
%           if first run, should equal arg `track`
% track_scale [m]

% convert to constraints
for i = 1:length(track.polygons)
    [track.polygons(i).A, track.polygons(i).b] = utils.vert2con(track.vertices(:,track.polygons(i).vertex_indices)');
end

track_new = struct;
track_new.vertices = nan(2,0);


debug_ = false;
if debug_
    figure(997)
    clf
    hold on
    axis equal
    xlim([-5 10])
    ylim([-10 5])
end

% find neighbor intersections
for i_p = 1:length(track.polygons)
    %% find shared vertices of directly neighboured polygons
    % leveraging track definition: each polygon has exactly two neighbours
    % with exactly on full shared edge each (equaling exactly two vertices
    % each)
    %
    % Subscripts
    %   p: current polygon to check
    %   f: next forward polygon neighbour
    %   b: next backward polygon neighbour
    
    %% initial: get enlarged current polygon
    p = vert2poly(get_track_polygon_vertices(i_p, track));
    if debug_; plot(p); end
    p_enlarged = vert2poly(enlarge_polygon_into_forward_direction(i_p, track, track_scale, direction_is_forward, debug_));
    if debug_; plot(p_enlarged); end
    
    %% repeat as long as forward neighbours lay in current polygon's enlarged region
    
    if direction_is_forward
        i_f = utils.mod1(i_p + 1, length(track.polygons));
    else
        i_f = utils.mod1(i_p - 1, length(track.polygons));
    end
    while true
        if debug_
            figure(997)
            clf
            hold on
            axis equal
            xlim([-5 10])
            ylim([-10 5])
            plot(p_enlarged)
        end

        p_next_neighbour = vert2poly(get_track_polygon_vertices(i_f, track));
        if debug_; plot(p_next_neighbour); end
        
        %% if next forward neighbour overlaps enlarged current polygon
        if p_enlarged.overlaps(p_next_neighbour)
            %% enlarge next forward neighbour
            p_next_neighbour_enlarged = vert2poly(enlarge_polygon_into_forward_direction(i_f, track, track_scale, direction_is_forward, debug_));
            if debug_; plot(p_next_neighbour_enlarged); end
            
            %% get overlap between current enlarged polygon and enlarged
            % neighbour
            p_overlap_enlarged_n_next_neighbour = p_enlarged.intersect(p_next_neighbour_enlarged);
            if debug_; plot(p_overlap_enlarged_n_next_neighbour); end
            
            %% divide current enlarged polygon by overlap with next
            % neighbour
            %   yields a polygon with two separated regions:
            %   on before overlap (backwards), one after (forwards)
%             p_enlarged_divided_by_neighbour_overlap = p_enlarged.subtract(p_overlap_enlarged_n_next_neighbour);
%             if debug_; plot(p_enlarged_divided_by_neighbour_overlap); end
%             %if debug_; scatter(p_enlarged_divided_by_neighbour_overlap.Vertices(:, 1), p_enlarged_divided_by_neighbour_overlap.Vertices(:, 2)); end
%             
%             % if subtract didn't work: probably numerical issues -->
%             % downscale p_enlarged a little bit to get proper separated
%             % polygons
%             if p_enlarged_divided_by_neighbour_overlap.NumRegions ~= 2
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
                        % NumRegions == 1 should be an edge-case to (for coarse tesselation), but
                        % never happened - enable if it happens sometime
                        assert(p_enlarged_divided_by_neighbour_overlap.NumRegions == 2, 'Numerical issues? Please investigate!')
                    end
    %             end


                    %% check which polygon is part of original polygon (= overlaps)
                    %   should be the first polygon due to track definition
                    tf_p_part_of_original = overlaps([p; p_enlarged_divided_by_neighbour_overlap.regions]);

                    %% choose original overlapping polygon
                    %   we want: polygon 2 and 3 (columns) relation to first polygon (row)
                    p_divided_selection_index = tf_p_part_of_original(1, 2:3);
                    temp = p_enlarged_divided_by_neighbour_overlap.regions;
                    p_enlarged_retain = temp(p_divided_selection_index);
                end
            if debug_; plot(p_enlarged_retain); end
            
            %% union: reatining old p_enlarged parts, add overlap with next neighbour
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
        else
            % no overlapping --> maximum convex enlargement reached
            break
        end
        
        % for-loop: forward index
        
        if direction_is_forward
            i_f = utils.mod1(i_f + 1, length(track.polygons));
        else
            i_f = utils.mod1(i_f - 1, length(track.polygons));
        end
    end
    if debug_; plot(p_enlarged, 'EdgeColor', 'r'); end
    %if debug_; drawnow; pause(0.2); end
    
    %% save output
    track_new.polygons(i_p).vertex_indices = length(track_new.vertices) + (1:length(poly2vert(p_enlarged)));
    track_new.vertices = [track_new.vertices  poly2vert(p_enlarged)];
    [track_new.polygons(i_p).A, track_new.polygons(i_p).b] = utils.vert2con(poly2vert(p_enlarged)');   
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
            axis equal
            xlim([-5 10])
            ylim([-10 5])
        end
        
        % get overlaps in both directions
        p_overlap_prev = vert2poly(get_track_polygon_vertices(i_p, track_overlapped));
        p_overlap_curr = vert2poly(get_track_polygon_vertices(i_p, track_new));
        if debug_; plot(p_overlap_prev); plot(p_overlap_curr); end
        
        % merge overlaps
        p_overlap = p_overlap_prev.union(p_overlap_curr);
        if debug_; plot(p_overlap, 'EdgeColor', 'r'); end
        
        %% save output
        track_merged_overlaps.polygons(i_p).vertex_indices = length(track_merged_overlaps.vertices) + (1:length(poly2vert(p_overlap)));
        track_merged_overlaps.vertices = [track_merged_overlaps.vertices  poly2vert(p_overlap)];
        [track_merged_overlaps.polygons(i_p).A, track_merged_overlaps.polygons(i_p).b] = utils.vert2con(poly2vert(p_overlap)');   
    end
    
    track_new = track_merged_overlaps;
end
end

function p_enlarged = enlarge_polygon_into_forward_direction(i_p, track, track_scale, direction_is_forward, debug_)
    % enlarge polygon into direction of shared edge with forward neighbour
    %
    % Inputs
    %   i_p: index of polygon to be expanded
    n_vertices_p = length(track.polygons(i_p).vertex_indices);
    
    % shared vertices with forward neighbour
    if direction_is_forward
        i_f = utils.mod1(i_p + 1, length(track.polygons)); % next forward polygon
    else
        i_f = utils.mod1(i_p - 1, length(track.polygons)); % next forward polygon
    end
    n_vertices_f = length(track.polygons(i_f).vertex_indices);
    [i_shared_vertices_f_index, ~] = find(repmat(track.polygons(i_p).vertex_indices, n_vertices_f, 1) == repmat(track.polygons(i_f).vertex_indices', 1, n_vertices_p));
    i_shared_vertices_f = track.polygons(i_f).vertex_indices(i_shared_vertices_f_index);
    assert(numel(i_shared_vertices_f) == 2, "number of shared vertices don't match, check your track creation!")
    shared_point_f_1 = track.vertices(:, i_shared_vertices_f(1));
    shared_point_f_2 = track.vertices(:, i_shared_vertices_f(2));
    if debug_; plot_vertices(get_track_polygon_vertices(i_p, track)'); plot_vertices(get_track_polygon_vertices(i_f, track)'); end
    
    %% find shared (opposite) constraints
    % forward
    b_p_f = create_b_of_edge(shared_point_f_1, shared_point_f_2);
    i_shared_constraints_p_f = find(abs(abs(track.polygons(i_p).b) - abs(b_p_f)) < 1e-10 * track_scale);
    assert(numel(i_shared_constraints_p_f) == 1, "number of vertices in constraint representation don't match! Please investigate");
   
    % just double check that neighbour has same amount of shared constraints
    i_shared_constraints_f_p = find(abs(abs(track.polygons(i_f).b) - abs(b_p_f)) < 1e-10 * track_scale);
    assert(numel(i_shared_constraints_f_p) == 1, "number of vertices in constraint representation don't match! Please investigate")
    
    %% expand current polygon into direction of shared edge
    % creating pseudo polygon with effectively shared edges expanded
    % that far that they become ineffective
    %   we can use the same index as polygon was converted to
    %   constraint representation with same function --> must be
    %   determinstically the same index order
%     factor_enlarging = 10000; % FIXME constant
    b_enlarged_p = track.polygons(i_p).b;
    % FIXME make max track limits dependent
    if b_enlarged_p(i_shared_constraints_p_f) > 0
        b_enlarged_p(i_shared_constraints_p_f) = 1e2;
        %b_enlarged_p(i_shared_constraints_p_f) = b_enlarged_p(i_shared_constraints_p_f) * factor_enlarging;
    else
        b_enlarged_p(i_shared_constraints_p_f) = 10;
        %b_enlarged_p(i_shared_constraints_p_f) = b_enlarged_p(i_shared_constraints_p_f) / factor_enlarging;
    end
    
    %% convert back to vertices representation
    p_enlarged = utils.con2vert(track.polygons(i_p).A, b_enlarged_p);
    %if debug_; plot_polygon(p_enlarged, ':'); end
    assert(~isempty(p_enlarged), 'Some error in polygon enlarging happened! Please investigate')
end

function b = create_b_of_edge(shared_point_1, shared_point_2)
    % create b of given line segment
    n = [0 -1; 1 0] * (shared_point_1 - shared_point_2);
    n = n ./ norm(n);
    b = n' * shared_point_1;
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


function plot_vertices(vertices, LineSpec)
    if nargin < 2; LineSpec = '-'; end
    %plot([vertices(1, :) vertices(1, 1)], [vertices(2, :) vertices(2, 1)], LineSpec)
    plot(vertices(1, :), vertices(2, :), LineSpec)
end