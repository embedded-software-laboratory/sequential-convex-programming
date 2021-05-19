function cfg_vh = vehicle_SCR(cfg)
cfg_vh = config.vehicle(cfg);
cfg_vh.approximation = cfg_vh.approximationSCR; %  % 'approximationSL' or 'approximationSL'
end