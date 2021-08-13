function handle = plot_vertices(vertices, LineSpec, varargin)
% plot vertices as polygon
if nargin < 2; LineSpec = '-'; end
%plot([vertices(1, :) vertices(1, 1)], [vertices(2, :) vertices(2, 1)], LineSpec)
handle = plot(vertices(1, :), vertices(2, :), LineSpec, varargin{:});
end