function p_poly = vert2poly(p_vert)
% convert from vertice representation to MATLAB's polyshape

% convert to correctly ordered polygon (current vertices are unordered)
p_vert = utils.poly.cleanse_convex_polygon(p_vert);
%if debug_; plot_polygon(p_vert); end
% ... so that we can convert to MATLAB's poly
p_poly = polyshape(p_vert(:, 1), p_vert(:, 2));
%if debug_; plot(p_poly); end
end