function cfg = scenario_endless_race(cfg)
% making race endless (or close to)
cfg.scn.description = [cfg.scn.description '\nwith endless race'];

cfg.race.n_laps = round(realmax);            % Number of laps to be driven
end