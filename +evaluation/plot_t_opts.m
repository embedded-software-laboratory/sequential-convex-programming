%% Plot simulation times of different vehicles
%% How to
%   1. load a "log.mat" from the output directory into workspace
%   2. run script

%% extract t_opts
t_opts = cell(1, length(log.vehicles));

% for every vehicle
for v = 1:length(log.vehicles)

    % NOTE: yields p.iterations * simulation_steps outputs
    controller_output = [log.vehicles{v}.controller_output];

    % squash into shape:
    % columns: controller iteration
    % rows: SCP iterations inside controller iteration
    t_opts{v} = reshape([controller_output.t_opt], [], length(log.vehicles{v}));
end

%% Statistics
stat = struct;
for v = 1:length(t_opts)
    % sum SCP iterations
    t_opt_per_iteration = sum(t_opts{v}, 1);
    
    % calc staistics
    stat(v).median = median(t_opt_per_iteration);
    stat(v).prctile_99 = prctile(t_opt_per_iteration, 99);
    stat(v).max = max(t_opt_per_iteration);
end

%% Plot
f = figure(9746);
clf(f)
set(f, 'Name', 'Optimization Time Statistics');

% vehicle (categories)
cats = cell(1, length(stat));
for v = 1:length(cats)
    cats{v} = sprintf('Vehicle %i', v);
end
cats = categorical(cats);

bar(cats, [[stat.median]' [stat.prctile_99]' [stat.max]']);
ylabel('Solver Time [s]')
legend('Median','99th Percentile','Maximum', 'location', 'northwest')

%% Display vehicle config
for v = 1:length(cfg.scn.vhs)
    fprintf('Vehicle %i\n', v);
    disp(cfg.scn.vhs{v})
end