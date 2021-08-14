function cfg_vh = vehicle_blocking(cfg_vh)
% adapts vehicle's apprxoimation to SCR
cfg_vh.description = [cfg_vh.description '\nwith blocking of other vehicles'];

cfg_vh.p.isBlockingEnabled = true;
end