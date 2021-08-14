function exportFigure(figure_number, filename)
% save given figure under given path (without file extension) as landscape
% PDF and MATLAB fig
set(figure_number, 'PaperOrientation', 'landscape');
print(['-f' num2str(figure_number)], [filename '.pdf'], '-dpdf', '-bestfit');
saveas(figure_number, [filename '.fig'], 'fig');