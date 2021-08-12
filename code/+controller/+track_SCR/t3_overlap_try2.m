function track_new = t3_overlap(track, track_scale, checkpoints)
% Inputs
% track
% track_scale [m]

debug_ = true;
if debug_
    figure(997)
    clf
    hold on
    axis equal
    xlim([-5 10])
    ylim([-10 5])
end

% convert track polygons to constraints
for i = 1:length(track.polygons)
    [track.polygons(i).A, track.polygons(i).b] = utils.vert2con(track.vertices(:,track.polygons(i).vertex_indices)');
end

% merge all track polygons to one polygon
poly_track = polyshape();
for i = 1:length(track.polygons)
    p_enlarged = track.vertices(:,track.polygons(i).vertex_indices);
    p_enlarged = cleanse_convex_polygon(p_enlarged);
    poly_track = union(poly_track, polyshape(p_enlarged(1, :), p_enlarged(2, :)));
end
if debug_; plot(poly_track); end

track_new = struct;
track_new.vertices = nan(2,0);

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
    n_vertices_p = length(track.polygons(i_p).vertex_indices);
    
    % shared vertices with forward neighbour
    i_f = utils.mod1(i_p + 1, length(track.polygons)); % next forward polygon
    n_vertices_f = length(track.polygons(i_f).vertex_indices);
    [i_shared_vertices_f_index, ~] = find(repmat(track.polygons(i_p).vertex_indices, n_vertices_f, 1) == repmat(track.polygons(i_f).vertex_indices', 1, n_vertices_p));
    i_shared_vertices_f = track.polygons(i_f).vertex_indices(i_shared_vertices_f_index);
    assert(numel(i_shared_vertices_f) == 2, "number of shared vertices don't match, check your track creation!")
    shared_point_f_1 = track.vertices(:, i_shared_vertices_f(1));
    shared_point_f_2 = track.vertices(:, i_shared_vertices_f(2));

    % shared vertices with backward neighbour
    i_b = utils.mod1(i_p - 1, length(track.polygons)); % next backward polygon
    n_vertices_b = length(track.polygons(i_b).vertex_indices);
    [i_shared_vertices_b_index, ~] = find(repmat(track.polygons(i_p).vertex_indices, n_vertices_b, 1) == repmat(track.polygons(i_b).vertex_indices', 1, n_vertices_p));
    i_shared_vertices_b = track.polygons(i_b).vertex_indices(i_shared_vertices_b_index);
    assert(numel(i_shared_vertices_b) == 2, "number of shared vertices don't match, check your track creation!")
    shared_point_b_1 = track.vertices(:, i_shared_vertices_b(1));
    shared_point_b_2 = track.vertices(:, i_shared_vertices_b(2));

    %% find shared (opposite) constraints
    % forward
    b_p_f = create_b_of_edge(shared_point_f_1, shared_point_f_2);
    i_shared_constraints_p_f = find(abs(abs(track.polygons(i_p).b) - abs(b_p_f)) < 1e-10 * track_scale);
    assert(numel(i_shared_constraints_p_f) == 1, "number of vertices in constraint representation don't match! Please investigate");
    % backward
    b_p_b = create_b_of_edge(shared_point_b_1, shared_point_b_2);
    i_shared_constraints_p_b = find(abs(abs(track.polygons(i_p).b) - abs(b_p_b)) < 1e-10 * track_scale);
    assert(numel(i_shared_constraints_p_b) == 1, "number of vertices in constraint representation don't match! Please investigate");
    
    % just double check that neighbour has same amount of shared constraints
    i_shared_constraints_f_p = find(abs(abs(track.polygons(i_f).b) - abs(b_p_f)) < 1e-10 * track_scale);
    assert(numel(i_shared_constraints_f_p) == 1, "number of vertices in constraint representation don't match! Please investigate")
    i_shared_constraints_b_p = find(abs(abs(track.polygons(i_b).b) - abs(b_p_b)) < 1e-10 * track_scale);
    assert(numel(i_shared_constraints_b_p) == 1, "number of vertices in constraint representation don't match! Please investigate")
    
    %%
        %% expand current polygon into direction of shared edge
        % creating pseudo polygon with effectively shared edges expanded
        % that far that they become ineffective
        %   we can use the same index as polygon was converted to
        %   constraint representation with same function --> must be
        %   determinstically the same index order
        factor_enlarging = 10000; % FIXME constant
        b_enlarged_p = track.polygons(i_p).b;
        % forward
        if b_enlarged_p(i_shared_constraints_p_f) > 0
            b_enlarged_p(i_shared_constraints_p_f) = b_enlarged_p(i_shared_constraints_p_f) * factor_enlarging;
        else
            b_enlarged_p(i_shared_constraints_p_f) = b_enlarged_p(i_shared_constraints_p_f) / factor_enlarging;
        end
        % backward
        if b_enlarged_p(i_shared_constraints_p_b) > 0
            b_enlarged_p(i_shared_constraints_p_b) = b_enlarged_p(i_shared_constraints_p_b) * factor_enlarging;
        else
            b_enlarged_p(i_shared_constraints_p_b) = b_enlarged_p(i_shared_constraints_p_b) / factor_enlarging;
        end

        %% conversions
        % convert back to polygon form
        p_enlarged = utils.con2vert(track.polygons(i_p).A, b_enlarged_p)';
        assert(~isempty(p_enlarged), 'Some error in polygon enlarging happened! Please investigate')
        %if debug_; plot_polygon(p_enlarged, ':'); end
        % convert to correctly ordered polygon (current vertices are unordered)
        p_enlarged = cleanse_convex_polygon(p_enlarged);
        %if debug_; plot_polygon(p_enlarged); end
        % ... so that we can convert to MATLAB's poly
        poly_enlarged = polyshape(p_enlarged(1, :), p_enlarged(2, :));
        %if debug_; plot(poly_enlarged); end

        %% create intersection between track and current expanded polygon
        %   equals union of current polygon and neihgbouring polygons'
        %   parts, which lay in the expansion of the shared vertice removed
        poly_intersections = intersect(poly_enlarged, poly_track);
        %if debug_; plot(intersection); end

        %% remove regions which are overshadowed by enlarged polygon
        %   (happening when reaching other parts of the track, which aren't
        %   connected to the original polygon)
        p_original = track.vertices(:,track.polygons(i_p).vertex_indices);
        poly_original = polyshape(p_original(1, :), p_original(2, :));
        if debug_; plot(poly_original); end
        if poly_intersections.NumRegions > 1
            poly_intersections_separated = regions(poly_intersections);
            %if debug_; plot(poly_intersections_separated); end
            poly_intersection_wanted_index = overlaps(poly_original, poly_intersections_separated);
            assert(sum(poly_intersection_wanted_index) == 1, 'More than on intersection matches! Please investigate')

            poly_overlapped = poly_intersections_separated(poly_intersection_wanted_index);
        else
            poly_overlapped = poly_intersections;
        end
        assert(poly_overlapped.NumRegions == 1, "Some error in intersection using MATLAB'S polyshapes! Please investigate")
        if debug_; plot(poly_overlapped); end
        
        
        
            vertices_overlapped_convex = poly_overlapped.Vertices';
        %% remove non-convex parts
        % if convex
%         if poly_overlapped.convhull.area == poly_overlapped.area
%             vertices_overlapped_convex = poly_overlapped.Vertices';
%         % if non-convex
%         else
%             % as we overlapped/intersected with whole track, we introduce
%             %   non-convex parts in S-Corners
%             %
%             %   starting with original polygon (which is convex and shall be
%             %   extended in a convex manner), we extend it by every point of
%             %   intersection. If the point extends original to a convex
%             %   polygon: extend. If not: it's a non-convex part --> trash
%             vertices_overlapped_convex = poly_original.Vertices';
%             for i = 1:length(poly_overlapped.Vertices)
%                 % previous iteration
%                 if debug_ && i ~= 1
%                     clf
%                     hold on
%                     axis equal
%                     plot(poly_original)
%                     plot(poly_overlapped)
%                     plot_polygon(vertices_overlapped_convex)
%                     plot_polygon(vertices_convex_to_check(:, vertices_conv_hull_indices), 'x--')
%                 end
%             
%                 % reinstaniate current state of found convex polygon
%                 vertices_convex_to_check = vertices_overlapped_convex;
%                 
%                 % current point to check if added to polygon if stays convex
%                 point_to_check = poly_overlapped.Vertices(i, :)';
% 
%                 % check point not within polygon
%                 [in, on] = inpolygon(point_to_check(1), point_to_check(2),...
%                                   vertices_convex_to_check(1, :), vertices_convex_to_check(2, :));
%                 
%                 % if already on polygon boundary: skip
%                 if on; continue; end
%                 assert(~in, 'Point is inside polygon! Please investigate')
%                 % extend convex polygon with point to check
%                 vertices_convex_to_check = [vertices_convex_to_check point_to_check]; %#ok<AGROW>
%                 if debug_; plot_polygon(vertices_convex_to_check); end
% 
%                 % 'Simplify' = true by default for polygons
%                 vertices_conv_hull_indices = convhull(vertices_convex_to_check');%, 'Simplify', true);
%                 vertices_conv_hull_indices_unused = setdiff(1:length(vertices_convex_to_check), vertices_conv_hull_indices');
%                 if debug_; plot_polygon(vertices_convex_to_check(:, vertices_conv_hull_indices)); end
%                 % if adding area, it must be convex. If
%                 % it must be convex, a new vertex has to be added, else it was
%                 % non-convex
%                 %   convhull gives out diplicated index at end
%                 %
%                 % if current point is added and keeps polygon convex
%                 if isempty(vertices_conv_hull_indices_unused)
%                     % replace poly with bigger one
%                     vertices_overlapped_convex = vertices_convex_to_check(:, vertices_conv_hull_indices);
%                 
%                 % check if non-convex or vertice only removed because
%                 % laying on edge between other vertices
%                 else
%                     for point_unused = vertices_convex_to_check(:, vertices_conv_hull_indices_unused)
%                         [in, on] = inpolygon(point_unused(1), point_unused(2),...
%                                       vertices_convex_to_check(1, :), vertices_convex_to_check(2, :));
% 
%                         % if excluded point lays on edge defined by other vertices: still convex
%                         if on
%                             % replace poly with bigger one
%                             vertices_overlapped_convex = vertices_convex_to_check(:, vertices_conv_hull_indices);
%                         % if non-convex (excluded point was vertice, but now is
%                         % inside --> non-convex polygon generated)
%                         elseif in && ~on
%                             %% try to add convex part of non-convex extension
%                             % slide around the two  lines
%                             %   [point_to_check currentConvexPolygon.nearestVertex]
%                             %   [point_to_check currentConvexPolygon.secondNearestVertex]
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% % FIXME add
%                             continue
%                         else
%                             error('should not be reachable! Please investigate')
%                         end
%                     end
%                 end
%             end
%         
%             if debug_
%                 clf
%                 hold on
%                 plot(poly_original)
%                 plot(poly_overlapped)
%                 plot_polygon(vertices_overlapped_convex)
%             end
%         end


        %% save output
        track_new.vertices = [track_new.vertices  vertices_overlapped_convex];
        %track_new.polygons(i1).vertex_indices = indices;
        [track_new.polygons(i_p).A, track_new.polygons(i_p).b] = utils.vert2con(vertices_overlapped_convex');        
    
    
%     %%
%     i_test = A1_test_ * track.vertices(:, track.polygons(i1).vertex_indices) == b1_test_;
%     b1_curr(i_test')
%     
%     %%
%     poly1 = polyshape([0 0 1 1],[1 0 0 1]);
%     poly2 = polyshape([1 1.25 1.25 1],[0.25 0.25 0.75 0.75]);
%     [in, on] = inpolygon(poly2.Vertices(:, 1),poly2.Vertices(:, 2), poly1.Vertices(:, 1), poly1.Vertices(:, 2))
%     sharedEdge = poly2.Vertices(on, :)
%     x1 = sharedEdge(1, 1);
%     x2 = sharedEdge(2, 1);
%     y1 = sharedEdge(1, 2);
%     y2 = sharedEdge(2, 2);
%     dx = x2 - x1; dy = y2 - y1;
%     normals = [-dy, dx];%[dy, -dx]
%     
%     poly2.Vertices(on, :) = poly2.Vertices(on, :) + normals * 2
    
%     %%
%     track.polygons(i_p).A_intersection = [A1;A2];
%     track.polygons(i_p).b_intersection = [b1;b2];
end

% track_new = struct;
% track_new.vertices = nan(2,0);
% 
% % add overlaps
% for i1 = 1:length(track.polygons)
% 
%     i0 = utils.mod1(i1-1, length(track.polygons));
%     i2 = utils.mod1(i1+1, length(track.polygons));
% 
%     vertices_0 = utils.con2vert([track.polygons(i0).A_intersection; track.polygons(i0).A], [track.polygons(i0).b_intersection; track.polygons(i0).b]);
%     vertices_1 = track.vertices(:, track.polygons(i1).vertex_indices)';
%     vertices_2 = utils.con2vert([track.polygons(i1).A_intersection; track.polygons(i2).A], [track.polygons(i1).b_intersection; track.polygons(i2).b]);
% 
%     vertices_union = [vertices_0; vertices_1; vertices_2];
% 
%     [~, area_0] = convhull(vertices_0);
%     [~, area_1] = convhull(vertices_1);
%     [~, area_2] = convhull(vertices_2);
%     [K, area_union] = convhull(vertices_union, 'simplify', true);
% 
%     % polygons need to be convex: close to same area
%     assert(abs(area_0 + area_1 + area_2 - area_union) < 1e-9 * (track_scale^2), 'overlapped polygons area mismatch');
%     % NOTE only track hockenheim requires slighlty larger deviation
%     % margin of 1e-8 - maybe was caused by missing scaling? then remove
%     % comment
% 
%     vertices_union = vertices_union(K(2:end),:);
% 
%     indices = size(track_new.vertices,2) + (1 : size(vertices_union, 1));
% 
%     track_new.vertices = [track_new.vertices  vertices_union'];
%     track_new.polygons(i1).vertex_indices = indices;
%     [track_new.polygons(i1).A, track_new.polygons(i1).b] = utils.vert2con(vertices_union);        
% end
end

function plot_polygon(vertices, LineSpec)
    if nargin < 2; LineSpec = '-'; end
    %plot([vertices(1, :) vertices(1, 1)], [vertices(2, :) vertices(2, 1)], LineSpec)
    plot(vertices(1, :), vertices(2, :), LineSpec)
end

function polygon = cleanse_convex_polygon(polygon)
    % order verticesm, remove unneccesary vertices
    select_and_reorder_indices = convhull(polygon');
    select_and_reorder_indices = select_and_reorder_indices(1:end-1);
    polygon = polygon';
    polygon = polygon(select_and_reorder_indices, :)';
end

function b = create_b_of_edge(shared_point_1, shared_point_2)
    % create b of given line segment
    n = [0 -1; 1 0] * (shared_point_1 - shared_point_2);
    n = n ./ norm(n);
    b = n' * shared_point_1;
end