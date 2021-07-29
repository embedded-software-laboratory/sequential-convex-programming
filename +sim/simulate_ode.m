function x_0_new = simulate_ode(x_0, u_1, veh_cfg)
% solve ode for individual vehicle with calculated inputs
% Inputs:
%   ws_vehicle (struct): working set of individual vehicle
%   veh_cfg (struct): vehicle configuration

if x_0(3) == 0
   x_0(3) = eps; 
end
sim_timestep = veh_cfg.p.dt/10; % Size of time step [s]
sim_dt = 0:sim_timestep:veh_cfg.p.dt; % Time span [s] -> n+ time points including n time steps
[~,sim_x_0] = ode15s(...
    @(t,x)...
    veh_cfg.model_simulation.ode(...
        x, u_1),... constant control input across simulation step
    sim_dt,...
    x_0,...
    odeset('RelTol',1e-8,'AbsTol',1e-8));
x_0_new = sim_x_0(end,:)';