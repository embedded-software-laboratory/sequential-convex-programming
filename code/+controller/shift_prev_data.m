function ws = shift_prev_data(n_vhs, ws)
% Forward controller data to new time-step
% advance by one step (with terminal constraints) yields:
for v = 1:n_vhs
    % Save trajectory for convex constraint plots
    ws.vhs{v}.X_controller_prev = ws.vhs{v}.X_controller;

    % keep last state entry
    % X_{n-1} = X_{n} due to terminal constraint = standstill
    ws.vhs{v}.X_controller(:, 1:end-1) = ws.vhs{v}.X_controller(:, 2:end);

    % zero last input
    % U_{n} = zeros() due to terminal constraint = standstill
    ws.vhs{v}.U_controller(:, 1:end-1) = ws.vhs{v}.U_controller(:, 2:end);
    ws.vhs{v}.U_controller(:, end) = 0 .* ws.vhs{v}.U_controller(:, end);
end