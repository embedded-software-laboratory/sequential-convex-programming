function cfg = scenario_n_laps_race(cfg, n_laps)
% making race endless (or close to)
cfg.scn.description = [cfg.scn.description '\nwith endless race'];

cfg.race.n_laps = n_laps;            % Number of laps to be driven
end