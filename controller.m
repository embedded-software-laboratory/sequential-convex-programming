% acceleration controller: converting from desired acceleration to single
% track inputs (namely torque and steering angle)


%% Parameters
p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
dyaw = 1;
v_x = 1;
v_y = 0;
% current acceleration values
a_x = 0.1;
a_y = 0;
    
options = optimoptions('fsolve','Display','off');
    
%% Test Loop
sum = 0;
for i = 1:1000
    tic
    f = @(x) accelerationEquations(x(1), x(2), p, dyaw, v_x, v_y, a_x, a_y);
    x0 = [0, 0];
    x = fsolve(f, x0, options);
    delta = x(1);
    torque = x(2);
    sum = sum + toc;
end
disp(sum);

function f = accelerationEquations(delta, torque, p, dyaw, v_x, v_y, a_x, a_y)
%     %% Extract Inputs
%     delta = x(1);
%     torque = x(2);
% 
%     %% Parameters
%     p = model.vehicle.SingleTrack.getParamsLinigerRC_1_43_WithLinigerBounds();
%     dyaw = 1;
%     v_x = 1;
%     v_y = 0;
%     % current acceleration values
%     a_x = 0.1;
%     a_y = 0;

    %% Tire forces
    % front/rear tire side slip angle
    alpha_f = -atan2(dyaw * p.l_f + v_y, v_x) + delta;
    alpha_r =  atan2(dyaw * p.l_r - v_y, v_x);
    % front/rear tire lateral force
    F_fy = p.Df * sin(p.Cf * atan(p.Bf * alpha_f));
    F_ry = p.Dr * sin(p.Cr * atan(p.Br * alpha_r));
    % rear tire longitudinal force
    F_rx = (p.Cm1 - p.Cm2 * v_x) * torque - p.Cr0 - p.Cr2 * v_x^2;

    %%
    % rearranged to fit form 0 = F(x)
    f(1) = -a_x + 1/p.m * (...
        F_rx...
        -...
        F_fy...
        * sin(delta) + p.m * v_y * dyaw);
    f(2) = -a_y + 1/p.m * (...
        F_ry...
        +...
        F_fy...
        * cos(delta) - p.m * v_x * dyaw);
end