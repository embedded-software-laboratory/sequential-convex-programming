function vertices = cleanse_convex_polygon(vertices)
% order vertices, remove unneccesary vertices
select_and_reorder_indices = convhull(vertices);
select_and_reorder_indices = select_and_reorder_indices(1:end-1);
vertices = vertices(select_and_reorder_indices, :);
end