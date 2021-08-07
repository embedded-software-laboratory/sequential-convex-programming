function cfg_vh = vehicle_SCR(cfg_vh)
% adapts vehicle's apprxoimation to SCR
cfg_vh.description = [cfg_vh.description '\nwith SCR track discretization'];

cfg_vh.approximation = cfg_vh.approximationSCR; %  % 'approximationSL' or 'approximationSL'
end