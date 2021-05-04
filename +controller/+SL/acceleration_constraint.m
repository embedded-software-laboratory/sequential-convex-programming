% Calculates a tangent to the elliptical acceleration constraints.
% p = parameter struct
% i = index of tangent
% x = [px,py,vx,vy] (previous state vector)

% Resulting constraint: Ax * [px,py,vx,vy] + Au * [ax,ay] <= b
function [Au, b] = acceleration_constraint(p,k,x)
    vx = x(3);
    vy = x(4);
    v_sq = vx^2 + vy^2;
    v = sqrt(v_sq);
    
    v_idx = p.v_idx(v);
    ay_max = p.a_lateral_max_list(v_idx);
    
    ax_max = p.a_forward_max_list(v_idx) .* ones(size(k));
    ax_max(p.acceleration_cos <= 0) = p.a_backward_max_list(v_idx);
    
    % simultaneously converting acceleration from global to vehicle reference frame
    % vectorized from
    %Au = zeros(length(i), 2);
    %  for j = 1:length(i)
    %      Au(j, :) = 1/sqrt(v_sq + 0.01) .* [ay_max*c(j) ax_max(j)*s(j)] * [vx vy; -vy vx];
    %  end
    Au = 1/sqrt(v_sq + 0.01) .* [...
        ay_max*p.acceleration_cos .* vx + ax_max.*p.acceleration_sin .* -vy...
        ay_max*p.acceleration_cos .* vy + ax_max.*p.acceleration_sin .* vx];
    b = ax_max * ay_max;
end