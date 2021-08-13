function p_vert = get_track_polygon_vertices(i, track)
% get vertices by index
p_vert = track.vertices(:, track.polygons(i).vertex_indices)';
end