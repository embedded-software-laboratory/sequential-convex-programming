function plot_vertices(vertices, LineSpec)
% plot vertices as polygon
if nargin < 2; LineSpec = '-'; end
%plot([vertices(1, :) vertices(1, 1)], [vertices(2, :) vertices(2, 1)], LineSpec)
plot(vertices(1, :), vertices(2, :), LineSpec)
end