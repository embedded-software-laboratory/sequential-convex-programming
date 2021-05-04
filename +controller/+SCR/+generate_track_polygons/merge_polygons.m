function track = merge_polygons(track)
% merge polygons to convex regions

% Iterate in reverse, because we will delete elements in the loop
for i = length(track.polygons):-1:2  
    idx_1 = track.polygons(i).vertex_indices;
    idx_2 = track.polygons(i-1).vertex_indices;
    % union indices of adjacent polygons
    idx_merged = unique([idx_1 idx_2]);

    [~, V1]       = convhull(track.vertices(:, idx_1)', 'simplify', true);
    [~, V2]       = convhull(track.vertices(:, idx_2)', 'simplify', true);
    [K, V_merged] = convhull(track.vertices(:, idx_merged)', 'simplify', true);

    % Only merge adjacent polygons if their union is (almost) convex.
    area_tolerance = .05;
    if V_merged - V1 - V2 < area_tolerance
        track.polygons(i) = []; % Delete first polygon
        track.polygons(i-1).vertex_indices = idx_merged(K(2:end)); % Replace second polygon with union
    if volume_merged - (volume_1 + volume_2) <= .0%5 % epsilon_volume_tolerance
    end
end
end