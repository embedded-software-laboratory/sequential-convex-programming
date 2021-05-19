function track = merge_polygons(track, track_scale, epsilon_area_tolerance)
% merge polygons to convex regions
%
% Inputs
% track
% track_scale [m]
% epsilon_area_tolerance [m^2]

% iterate in reverse: deleting polygons in the loop
for i = length(track.polygons):-1:2
    % construct polygon indices
    idx_1 = track.polygons(i).vertex_indices;
    idx_2 = track.polygons(i-1).vertex_indices;
    % union indices of adjacent polygons
    idx_union = union(idx_1, idx_2);

    % calculate hull areas & indices
    [~, area_1]                 = convhull(track.vertices(:, idx_1)');
    [~, area_2]                 = convhull(track.vertices(:, idx_2)');
    % simplify: only keep hull vertices
    [indices_hull, area_merged] = convhull(track.vertices(:, idx_union)', 'simplify', true);

    % merge polygons if their union is (almost) convex (almost same area)
    if area_merged - (area_1 + area_2) <= epsilon_area_tolerance * (track_scale^2)
        % delete polygon 1
        track.polygons(i) = [];
        % replace polygon 2 with union (first and last entry of hull is duplicated)
        track.polygons(i-1).vertex_indices = idx_union(indices_hull(2:end));
    end
end
end