function polygon = cleanse_convex_polygon(polygon)
% order vertices, remove unneccesary vertices
select_and_reorder_indices = convhull(polygon);
select_and_reorder_indices = select_and_reorder_indices(1:end-1);
polygon = polygon(select_and_reorder_indices, :);
end