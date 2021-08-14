function exportFigure(figure_number, filename)
% save given figure under given path (without file extension) as landscape
% PDF and MATLAB fig
set(figure_number, 'PaperOrientation', 'landscape');
print(['-f' num2str(figure_number)], [filename '.pdf'], '-dpdf', '-bestfit');

warning off % gives redundant warning about using savefig
savefig(figure_number, [filename '.fig'], 'compact');
warning on