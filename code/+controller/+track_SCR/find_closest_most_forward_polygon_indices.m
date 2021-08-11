function polygon_indices = find_closest_most_forward_polygon_indices(positions, polygons, Hp)
% For each trajectory point, find the most forward track polygon. As a
%   fallback, the closest polygon is given out (e.g., in case of slackened
%   track limits or numerical issues

polygon_indices = nan(Hp, 1);

for k = 1:Hp
    %% find closest polygon to and polygons containing position
    
    distance_to_polygon_min = Inf;
    % polygons containing position
    containing_polygons_indices = [];

    for j = 1:length(polygons)
        % distance to edge of polygon
        %   max because polygon is in halfspace-represenation ->
        %   negative: inside polygon; positive: outside
        %       case negative: edge which is closest
        %       case positive: edge which is furthest away
        distance_to_polygon = max(polygons(j).A * positions(:, k) - polygons(j).b);
        
        % save closest polygon
        if distance_to_polygon_min > distance_to_polygon
            distance_to_polygon_min = distance_to_polygon;
            distance_to_polygon_min_index = j;
        end
        
        % save polygon if containing position
        if distance_to_polygon <= 0
            containing_polygons_indices(end + 1) = j; %#ok<AGROW>
        end
        
        % debug: plot every polygon and position
        %figure(998)
        %clf
        %plot(polygon_vertices(1, :), polygon_vertices(2, :)) % polygon
        %axis equal
        %hold on
        %plot(positions(1, k), positions(2, k), 'r+') % position
    end
    
    %% Save closest or most forward polygon index
    % if any position-containing polygons found: choose most forward
    % one
    if ~isempty(containing_polygons_indices)
        % calculate differences between polygons
        %   wrapping around from nth to 1st polygon (with absolute
        %   difference)
        % distance between each polygon in [#polygons]
        distance_between_polygon_indices = diff(...
            [containing_polygons_indices...
             containing_polygons_indices(1) + length(polygons)]);
        
        % we define last polygon cointaing position as the one having the
        %   largest distance to the next polygon in [#polygons]
        most_forward_polygon_index_index = find(distance_between_polygon_indices == max(distance_between_polygon_indices), 1, 'last');
        
        % extract 
        polygon_indices(k) = containing_polygons_indices(most_forward_polygon_index_index);
    else
        warning("position couldn't be located inside any polygon. Choosing closest polygon instead. (This can happen if track area was slackened or due to numerical issues)")
        polygon_indices(k) = distance_to_polygon_min_index;
    end
end