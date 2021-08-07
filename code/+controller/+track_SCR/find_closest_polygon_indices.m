function pg_indices = find_closest_polygon_indices(position, pg, Hp)
% For each trajectory point, find the closest track polygon.
pg_indices = nan(Hp, 1);

for k = 1:Hp
    min_signed_distance = 1e300;
    argmin_signed_distance = 0;

    for j = length(pg):-1:1
        signed_distance = max(pg(j).A * position(:, k) - pg(j).b);
        if min_signed_distance > signed_distance
            min_signed_distance = signed_distance;
            argmin_signed_distance = j;
        end
    end
    pg_indices(k) = argmin_signed_distance;
end
end