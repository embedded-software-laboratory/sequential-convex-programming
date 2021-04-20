% possible iterates:
% - Iterations =1, =2
% - Alle Modelle
% - Alle controller
% - #vehicles
% - All tracks
% - Blocking etc dis/enabled
% cfg_vehicle.p.isBlockingEnabled = false;
% cfg_vehicle.p.areObstaclesConsidered = false;


%% build iterates
iterates = {};
% tracks
iterates{end+1} = {'cfg.scn.track';
    track.hockenheim_record(3, 9.4);
    % track.testtrack_1(20);
    % track.hockenheim_simple();
    % track.Hockenheim1();
    % track.Hockenheim2();
    % track.Hockenheim3();
    % track.Hockenheim4();
    % track.testTrack1();
    % track.testTrack2();
    % track.testTrack3();
    % track.testTrack4();
    % track.testTrack5();
    % track.testTrack6();
    % track.testTrack7();
    % track.testTrack8();
    % track.testTrack9();
    track.testTrack10()};

% iterations
iterates{end+1} = {'cfg_vehicle.p.iterations';
    1;
    2};


%% iterate runs
% iterate parameter studys
for i=1:length(iterates)
    iterate = iterates{i};
    
    % iterate parameters of parameter study
    for j=2:length(iterate)
        % load default cfg
        cfg = config();
        cfg = config_scenario(cfg);
        cfg.race.n_laps = 1;
        cfg.plot.is_enabled = false;
    
        % update cfg
        eval([iterate{1}, ' = iterate{j}'])
        
        % init config
        cfg = init_config(cfg);
    
        % run with given config
        run(cfg);
    end
end
