function cfg_vh = vehicle_avoidance(cfg_vh)
% adapts vehicle's apprxoimation to SCR
cfg_vh.description = [cfg_vh.description '\nwith collision avoidance'];

cfg_vh.p.areObstaclesConsidered = true;
if cfg_vh.p.SCP_iterations < 2
    cfg_vh.p.SCP_iterations = 2;
end
end