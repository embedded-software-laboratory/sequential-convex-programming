function exportFigure(figure_number, filename)
% save given figure under given path (without file extension) as landscape PDF
set(figure_number, 'PaperOrientation', 'landscape');
print(['-f' num2str(figure_number)], [filename '.pdf'], '-dpdf', '-bestfit');