% Calculates a tangent to the elliptical acceleration constraints.
% p = parameter struct
% i = index of tangent
% x = [px,py,vx,vy] (previous state vector)

% Resulting constraint: Ax * [px,py,vx,vy] + Au * [ax,ay] <= b
function [Au, b] = acceleration_constraint_tangent(p,i,x)
    vx = x(3);
    vy = x(4);
    v_sq = vx^2 + vy^2;
    v = sqrt(v_sq);
    
    c = cos(i*p.delta_angle);
    s = sin(i*p.delta_angle);
    
    v_idx = p.v_idx(v);
    ay_max = p.a_lateral_max_list(v_idx);    
    if c > 0
        ax_max = p.a_forward_max_list(v_idx);
    else
        ax_max = p.a_backward_max_list(v_idx);
    end
    
    Au = 1/sqrt(v_sq + 0.01) * [ay_max*c ax_max*s] * [vx vy; -vy vx];
    b = ax_max * ay_max;
end