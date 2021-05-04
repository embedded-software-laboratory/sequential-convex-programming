% "Should" calculate inner n-polygon of circle
% p = parameter struct
% i = index of tangent

% Resulting constraint: Ax * [px,py,vx,vy] + Au * [ax,ay] <= b
function [Au, b] = acceleration_constraint(p,k,~)
     Au = [cos(2*pi.*k/p.n_acceleration_limits) sin(2*pi.*k/p.n_acceleration_limits)];
     b = p.a_max;
end