function cfg = scenario_1_vehicle(cfg, vehicle_config_handle)
% wrapper for a scenario with one givenvehicle
cfg.scn.description = [cfg.scn.description '\nwith 1 vehicle only'];

cfg.scn.vhs{end + 1} = vehicle_config_handle(config.base_vehicle(cfg));
end