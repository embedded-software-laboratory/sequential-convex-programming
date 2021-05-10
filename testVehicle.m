%% Config
dt = 0.01; % [s]
t_end = 50; % [s]


%% Simulate
t_end_index = t_end / dt;
Hp = 1; % arbitrarym not used here

m = model.vehicle.SingleTrack(Hp, dt, model.vehicle.SingleTrack.getParamsLinigerRC_1_43());

t_sim = (0:t_end_index) * dt;

x = NaN(length(t_sim), 6);
x(1, :) = [0 0 0.5 0 0 0]; % initial state

u = NaN(length(t_sim), 2);
% Constant max power
u(:, 1) = 0; % delta
u(:, 2) = 1; % torque

a_measured = NaN(length(t_sim), 2); % a_x a_y

t = 0;
for i = 2:length(t_sim)    
    % simulate model from last timestep for current
    [~, x_sim] = ode15s(@(t, x) m.ode(x, u(i, :)), [t_sim(i - 1) t_sim(i)], x(i - 1, :));
    x(i, :) = x_sim(end, :);
    
    a_measured(i, :) = (x(i, 3:4) - x(i-1, 3:4)) / dt;
end

%% Plot
close all
figure('Name', 'v_x');
plot(t_sim, x(:, 3))
xlabel('t [s]');

figure('Name', 'a (measured)');
plot(t_sim, a_measured)
xlabel('t [s]');
legend('a_x', 'a_y');

figure('Name', 'a (measured) over v_x');
plot(x(:, 3), a_measured)
xlabel('v_x [m/s]');
legend('a_x', 'a_y');