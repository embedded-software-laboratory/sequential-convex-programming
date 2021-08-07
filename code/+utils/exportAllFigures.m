function exportAllFigures(cfg)
% save all figures as PDF to results folder
all_figures = findall(groot,'Type','figure');
for i = 1:length(all_figures)
    filename = ['Figure ' num2str(all_figures(i).Number) ' - ' all_figures(i).Name];
    filename = utils.sanitizeFilename(filename);
    
    % save figure
    set(all_figures(i).Number, 'PaperOrientation', 'landscape');
    print(['-f' num2str(all_figures(i).Number)], [cfg.outputPath filename], '-dpdf', '-bestfit');
end