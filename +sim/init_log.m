function log = init_log(ws, cfg)
% initialize logging structure on basis of given working set

% Initialize logging structure
fields = fieldnames(ws)';
fields(ismember(fields, 'vhs')) = [];
fields{2,1} = {};
log.lap = struct(fields{:});

% copy structure from ws
fields = fieldnames(ws.vhs{1})';
fields{2,1} = {};
for i = 1:length(cfg.scn.vhs)
    log.vehicles{i} = struct(fields{:});   % Struct to save all data for each lap
end