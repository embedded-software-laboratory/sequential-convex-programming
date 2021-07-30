function ws = init_ws(cfg)
% initialize working structure

ws = struct;
% vehicle-specfic working set
for i = 1:length(cfg.scn.vhs)
    % controller-specifics
    ws.vhs{i}.controller_output = NaN;
    ws.vhs{i}.x_0 = cfg.scn.vhs{i}.x_start;
    if cfg.scn.vhs{i}.isControlModelLinear && ~cfg.scn.vhs{i}.isSimulationModelLinear 
        % convert states from Single Track to Linear
        ws.vhs{i}.x_0_controller = model.vehicle.state_st2lin(ws.vhs{i}.x_0);
    else
        ws.vhs{i}.x_0_controller = ws.vhs{i}.x_0;
    end
    ws.vhs{i}.X_controller = repmat(ws.vhs{i}.x_0_controller, 1, cfg.scn.vhs{i}.p.Hp);
    ws.vhs{i}.U_controller = repmat([0;0], 1, cfg.scn.vhs{i}.p.Hp);
    ws.vhs{i}.u_1 = ws.vhs{i}.U_controller(:, 1);

    % lap-specific
    cp_x_0 = utils.find_closest_track_checkpoint_index(...
        ws.vhs{i}.x_0(cfg.scn.vhs{i}.model_controller.idx_pos), cfg.scn.track_center, 1);
    ws.vhs{i}.cp_prev = cp_x_0;
    ws.vhs{i}.cp_curr = cp_x_0;
    ws.vhs{i}.lap_count = -1; % start with 0 finished laps (but -1 because vehicles start just short of finish line)
    ws.vhs{i}.pos = 0; % ego vehicle position relative to all other vehicles
end

% inter-vehicle/obstacle data
%   Initialize tables with indications for each vehicle (rows) which other
%   vehicle has to be considered as an obstacle (colums) or wich other
%   vehicle has to be blocked (columns)
ws.obstacleTable = zeros(length(cfg.scn.vhs), length(cfg.scn.vhs));
ws.blockingTable = zeros(length(cfg.scn.vhs), length(cfg.scn.vhs));

% init data (in simulation: executed every round)
ws = sim.update_administrative_data(cfg, ws);