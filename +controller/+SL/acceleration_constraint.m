% Calculates a tangent to the elliptical acceleration constraints.
% p = parameter struct
% i = index of tangent
% x = [px,py,vx,vy] (previous state vector)

% Resulting constraint: Ax * [px,py,vx,vy] + Au * [ax,ay] <= b
function [Au, b] = acceleration_constraint(p,k,x)

    vx = x(3);
    vy = x(4);
    v_sq = vx*vx + vy*vy;
    v = sqrt(v_sq);
    
    delta_angle = 2*pi / p.n_acceleration_limits;
    
    c = cos(k*delta_angle); %warum versucht auf 0 zu ändern?
    s = sin(k*delta_angle); %warum versucht auf 1 zu ändern?
    
    v_idx = p.v_idx(v);
    ay_max = p.a_lateral_max_list(v_idx);
    ax_forward_max = p.a_forward_max_list(v_idx);
    ax_backward_max = p.a_backward_max_list(v_idx);
    
    if c > 0
        ax_max = ax_forward_max;
    else
        ax_max = ax_backward_max;
    end
    
    % simultaneously converting acceleration from global to vehicle reference frame
    Au = 1/sqrt(v_sq + 0.01) * [ay_max*c ax_max*s] * [vx vy; -vy vx];
    b = ax_max * ay_max;
end