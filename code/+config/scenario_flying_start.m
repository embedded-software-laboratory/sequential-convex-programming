function cfg = scenario_flying_start(cfg)
    % making race endless (or close to)
    cfg.scn.description = [cfg.scn.description '\nwith flying start'];
    
    [x_0, X_controller_start] = config.compute_ST_flying_initialization();

    for i = 1:numel(cfg.scn.vhs)
        cfg.scn.vhs{i}.x_start = x_0;
        cfg.scn.vhs{i}.X_controller_start = X_controller_start;
    end
end