function polygon_indices = find_closest_most_forward_polygon_indices(positions, track, Hp)
% For each trajectory point, find the most forward track polygon. As a
%   fallback, the closest polygon is given out (e.g., in case if outside
%   track limits or numerical issues)

polygon_indices = nan(Hp, 1);

for k = 1:Hp
    polygon_indices(k) = controller.track_SCR.find_closest_most_forward_polygon_index(positions(:, k), track);
end