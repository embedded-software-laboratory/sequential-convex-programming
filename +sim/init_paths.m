function init_paths(cfg)
% create output & temp dir if non-existing
if ~isfolder(cfg.outputPath); mkdir(cfg.outputPath); end
if ~isfolder(cfg.tempPath); mkdir(cfg.tempPath); end