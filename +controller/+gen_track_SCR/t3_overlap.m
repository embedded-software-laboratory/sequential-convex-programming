function track_new = t3_overlap(track, track_scale)
% Inputs
% track
% track_scale [m]

% convert to constraints
for i = 1:length(track.polygons)
    [track.polygons(i).A,track.polygons(i).b] = utils.vert2con(track.vertices(:,track.polygons(i).vertex_indices)');
end

% find neighbor intersections
for i1 = 1:length(track.polygons)
    % find shared vertices
    i2 = utils.mod1(i1+1, length(track.polygons));
    nv1 = length(track.polygons(i1).vertex_indices);
    nv2 = length(track.polygons(i2).vertex_indices);
    [I,~] = find(repmat(track.polygons(i1).vertex_indices, nv2, 1) == repmat(track.polygons(i2).vertex_indices', 1, nv1));
    shared_vertices = track.polygons(i2).vertex_indices(I);
    assert(numel(shared_vertices) == 2);

    % find shared (opposite) constraints
    p1 = track.vertices(:, shared_vertices(1));
    p2 = track.vertices(:, shared_vertices(2));

    n = [0 -1;1 0]* (p1-p2);
    n = n ./ norm(n);
    b = n'*p1;
    I1 = find(abs(abs(track.polygons(i1).b) - abs(b)) < 1e-10 * track_scale);
    I2 = find(abs(abs(track.polygons(i2).b) - abs(b)) < 1e-10 * track_scale);
    assert(numel(I1) == 1);
    assert(numel(I2) == 1);

    % remove shared constraints
    A1 = track.polygons(i1).A;
    b1 = track.polygons(i1).b;
    A2 = track.polygons(i2).A;
    b2 = track.polygons(i2).b;

    A1(I1,:) = [];
    b1(I1,:) = [];
    A2(I2,:) = [];
    b2(I2,:) = [];

    track.polygons(i1).A_intersection = [A1;A2];
    track.polygons(i1).b_intersection = [b1;b2];
end

track_new = struct;
track_new.vertices = nan(2,0);

% add overlaps
for i1 = 1:length(track.polygons)

    i0 = utils.mod1(i1-1, length(track.polygons));
    i2 = utils.mod1(i1+1, length(track.polygons));

    vertices_0 = utils.con2vert([track.polygons(i0).A_intersection; track.polygons(i0).A], [track.polygons(i0).b_intersection; track.polygons(i0).b]);
    vertices_1 = track.vertices(:, track.polygons(i1).vertex_indices)';
    vertices_2 = utils.con2vert([track.polygons(i1).A_intersection; track.polygons(i2).A], [track.polygons(i1).b_intersection; track.polygons(i2).b]);

    vertices_union = [vertices_0; vertices_1; vertices_2];

    [~, area_0] = convhull(vertices_0);
    [~, area_1] = convhull(vertices_1);
    [~, area_2] = convhull(vertices_2);
    [K, area_union] = convhull(vertices_union, 'simplify', true);

    % polygons need to be convex: close to same area
    assert(abs(area_0 + area_1 + area_2 - area_union) < 1e-9 * (track_scale^2));
    % FIXME only track hockenheim requires slighlty larger deviation
    % margin of 1e-8 - maybe was caused by missing scaling? then remove
    % comment

    vertices_union = vertices_union(K(2:end),:);

    indices = size(track_new.vertices,2) + (1 : size(vertices_union, 1));

    track_new.vertices = [track_new.vertices  vertices_union'];
    track_new.polygons(i1).vertex_indices = indices;
    [track_new.polygons(i1).A, track_new.polygons(i1).b] = utils.vert2con(vertices_union);        
end
end