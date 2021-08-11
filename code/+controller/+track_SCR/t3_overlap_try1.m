function track_new = t3_overlap(track, track_scale)
% Inputs
% track
% track_scale [m]

% convert to constraints
for i = 1:length(track.polygons)
    [track.polygons(i).A,track.polygons(i).b] = utils.vert2con(track.vertices(:,track.polygons(i).vertex_indices)');
end

track_new = track;
track_new.vertices = nan(2,0);

for i_p = 1:length(track.polygons)
    % starting from i1 (current polygon), first overlap polygons backwards,
    % then forwards
    i_ss.f = ...
        [i_p+1:+1:min(i_p+length(track.polygons)/2, length(track.polygons))... forward
        1:+1:i_p+length(track.polygons)/2 - length(track.polygons)]; % forward remaining, if required
 
    i_ss.b = ...
        [i_p-1:-1:max(i_p-length(track.polygons)/2, 1)... backward
         length(track.polygons):-1:length(track.polygons) + i_p-length(track.polygons)/2]; % backward remaining, if required
    assert(length(i_ss.f) == length(i_ss.b))
    
    for i_s = 1:length(i_ss.f)
        %% find neighbor intersections
        % find shared vertices
        %i2 = utils.mod1(i1+1, length(track.polygons));
        nv_p = length(track_new.polygons(i_p).vertex_indices);
        nv_pf = length(track.polygons(i_ss(i_s).f).vertex_indices);
        nv_pb = length(track.polygons(i_ss(i_s).f).vertex_indices);
        
        [I_f,~] = find(repmat(track_new.polygons(i_p).vertex_indices, nv_pf, 1) == repmat(track.polygons(i_ss.f(i_s)).vertex_indices', 1, nv_p));
        shared_vertices_f = track.polygons(i_ss(i_s).f).vertex_indices(I_f);
        [I_b,~] = find(repmat(track_new.polygons(i_p).vertex_indices, nv_pb, 1) == repmat(track.polygons(i_ss.b(i_s)).vertex_indices', 1, nv_p));
        shared_vertices_b = track.polygons(i_ss(i_s).b).vertex_indices(I_b);
        
        % if polygons are direct neighbours: must have shared vertices per
        % creation
        if i_p + 1 == i_ss(i_s).f
            assert(numel(shared_vertices_f) == 2, "direct polygon neighbours don't have shared vertices, but should have! Check your track creation");
        elseif utils.mod1(i_p-1, length(track.polygons)) == i_ss(i_s).b
            assert(numel(shared_vertices_b) == 2, "direct polygon neighbours don't have shared vertices, but should have! Check your track creation");
        elseif numel(shared_vertices_f) == 0
            continue
        else
            warning('Strange number of shared vertices, please investigate')
            continue
        end

        % find shared (opposite) constraints
        p1 = track.vertices(:, shared_vertices_f(1));
        p2 = track.vertices(:, shared_vertices_f(2));

        n = [0 -1;1 0]* (p1-p2);
        n = n ./ norm(n);
        b = n'*p1;
        I1 = find(abs(abs(track.polygons(i_p).b) - abs(b)) < 1e-10 * track_scale);
        I2 = find(abs(abs(track.polygons(i2_forwards).b) - abs(b)) < 1e-10 * track_scale);
        assert(numel(I1) == 1);
        assert(numel(I2) == 1);

        % remove shared constraints
        A1 = track.polygons(i_p).A;
        b1 = track.polygons(i_p).b;
        A2 = track.polygons(i2_forwards).A;
        b2 = track.polygons(i2_forwards).b;

        A1(I1,:) = [];
        b1(I1,:) = [];
        A2(I2,:) = [];
        b2(I2,:) = [];

        track.polygons(i_p).A_intersection = [A1;A2];
        track.polygons(i_p).b_intersection = [b1;b2];
        
        
        %% add overlaps

        vertices_b = utils.con2vert([track.polygons(i0).A_intersection; track.polygons(i0).A], [track.polygons(i0).b_intersection; track.polygons(i0).b]);
        vertices_p = track.vertices(:, track.polygons(i_p).vertex_indices)';
        vertices_f = utils.con2vert([track.polygons(i_p).A_intersection; track.polygons(i2_forwards).A], [track.polygons(i_p).b_intersection; track.polygons(i2_forwards).b]);

        vertices_union = [vertices_b; vertices_1; vertices_f];

        [~, area_b] = convhull(vertices_b);
        [~, area_p] = convhull(vertices_p);
        [~, area_f] = convhull(vertices_f);
        [K, area_union] = convhull(vertices_union, 'simplify', true);

        % polygons need to be convex: close to same area
        assert(abs(area_b + area_p + area_f - area_union) < 1e-9 * (track_scale^2), 'overlapped polygons area mismatch');
        % NOTE only track hockenheim requires slighlty larger deviation
        % margin of 1e-8 - maybe was caused by missing scaling? then remove
        % comment

        vertices_union = vertices_union(K(2:end),:);

        indices = size(track_new.vertices, 2) + (1 : size(vertices_union, 1));

        track_new.vertices = [track_new.vertices  vertices_union'];
        track_new.polygons(i_p).vertex_indices = indices;
        [track_new.polygons(i_p).A, track_new.polygons(i_p).b] = utils.vert2con(vertices_union);   
        
        % debug: plot every polygon and position
        debug_ = true;
        if debug_
            figure(997)
            clf
            axis equal
            hold on
            plot_polygon(vertices_p') % polygon
            plot_polygon(vertices_f', ':') % polygon
            plot_polygon(vertices_union', '--') % polygon
            legend('poly current', 'poly to check', 'union')
        end
    end
end


% for i1 = 1:length(track.polygons)
%     for i2 = 1:length(track.polygons)     
%     end
% end

% track_new_new = struct;
% for k = 1:length(track.polygons)
%     % pre-set last polygon for loop around between nth and 1st element
%     polyshape_curr = polyshape(track.vertices(:, track.polygons(length(track.polygons)).vertex_indices)');
%     for i = 1:length(track.polygons)
%         % skip if same polgon
%         if i == k; continue; end
% 
%         % convert to polyshape to use MATLAB's built in polygon functions
%         polyshape_to_check = polyshape(track.vertices(:, track.polygons(i).vertex_indices)');
% 
%         poly_intersection = intersect(polyshape_curr, polyshape_to_check);
%         
%         
%         % debug: plot every polygon and position
%         debug_ = true;
%         if debug_
%             figure(997)
%             clf
%             plot_polygon(polyshape_curr.Vertices') % polygon
%             axis equal
%             hold on
%             plot_polygon(polyshape_to_check.Vertices') % polygon
%             legend('poly current', 'poly to check')
%         end
% 
%         % if intersection exists
%         if poly_intersection.NumRegions > 0
%             % expand polygon with intersection
%             polyshape_curr = union(polyshape_curr, poly_intersection);
%         
%             if debug_
%                 plot_polygon(poly_intersection.Vertices') % polygon
%                 plot_polygon(polyshape_curr.Vertices') % polygon
%                 legend('poly current', 'poly to check', 'intersection', 'poly merged')
%             end
%         end
%     end
%     
%     % save polygon
%     track_new_new.polygons(k).vertices = polyshape_curr.Vertices';
%     % pre-compute for speed during controller execution
%     [track_new_new.polygons(k).A, track_new_new.polygons(k).b] = utils.vert2con(track_new_new.polygons(k).vertices);
% end
end

function plot_polygon(vertices, LineSpec)
    if nargin < 2; LineSpec = '-'; end
    %plot([vertices(1, :) vertices(1, 1)], [vertices(2, :) vertices(2, 1)], LineSpec)
    plot(vertices(1, :), vertices(2, :), LineSpec)
end