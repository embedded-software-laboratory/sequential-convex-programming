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
p_enlarged = utils.poly.con2vert(track.polygons(i_p).A, b_enlarged_p);
%if debug_; plot_polygon(p_enlarged, ':'); end
assert(~isempty(p_enlarged), 'Some error in polygon enlarging happened! Please investigate')
end

function b = create_b_of_edge(shared_point_1, shared_point_2)
    % create b of given line segment
    n = [0 -1; 1 0] * (shared_point_1 - shared_point_2);
    n = n ./ norm(n);
    b = n' * shared_point_1;
end