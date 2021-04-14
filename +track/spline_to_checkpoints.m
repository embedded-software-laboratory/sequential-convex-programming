function checkpoints = spline_to_checkpoints(spline, trackWidth, N)
point_coeffs = zeros(4,N);
tangent_coeffs = zeros(4,N);

for k = 1:N
    t = k/N;
    point_coeffs(1,k) = (1+2*t)*(1-t)^2;
    point_coeffs(2,k) = t*(1-t)^2;
    point_coeffs(3,k) = t^2*(3-2*t);
    point_coeffs(4,k) = t^2*(t-1);
        
    tangent_coeffs(1,k) = 6*t^2 - 6*t;
    tangent_coeffs(2,k) = 3*t^2 - 4*t +1;
    tangent_coeffs(3,k) = -6*t^2 + 6*t;
    tangent_coeffs(4,k) = 3*t^2 - 2*t;
end

checkpoints = struct;
for k = 1:length(spline)-1
    
    B = [spline(k).point 2*spline(k).tangent spline(k+1).point 2*spline(k+1).tangent];
    points = B * point_coeffs;
    tangents = B * tangent_coeffs;
    
    for j = 1:N
        % vector tangential to movement diraction, normalized
        t = tangents(:,j);
        t = t / norm(t);
        
        % normal vector, pointing to the left (called "n" in paper)
        n = [0 -1;1 0] * t;
        
        % save discretization in struct
        checkpoints(end + 1).forward_vector = t; % end +1 was (k-1)*N+j
        checkpoints(end).normal_vector = n;
        checkpoints(end).center = points(:,j);
        checkpoints(end).left = points(:,j) + trackWidth * n/2;        
        checkpoints(end).right = points(:,j) - trackWidth * n/2;
    end
end
end

