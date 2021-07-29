function cfg = scenario_endless_race(cfg)
% making race endless (or close to)
cfg.race.n_laps = round(realmax);            % Number of laps to be driven
end