function x_0 = simulate_ode(x_0, u_1, veh_cfg)
% solve ode for individual vehicle with calculated inputs
%   constant control input across simulation step
%
% Inputs
%   x_0    current simulation's states
%   u_1         current control inputs
%   veh_cfg     (struct) vehicle configuration
%
% Outputs
%   x_0    next simulation's states, based on input and last state

% prevent division errors if low v_{long}
if x_0(3) == 0
   x_0(3) = eps; 
end

% time span [s] -> n+ time points including n time steps
%   CAVE actually, MATLAB's ODE solvers don't care about these steps
dt_sim = 0:veh_cfg.p.dt_simulation:veh_cfg.p.dt_controller; 

% create function handle: varying states in sim, but keeping inputs constant
%   if simulation modell has controller:
%       use controller to translate between control output and sim input
if ~ismethod(veh_cfg.model_simulation, 'controller')
    u_1_for_sim = u_1;
    function_sim = @(t, x_in_sim) veh_cfg.model_simulation.ode(x_in_sim, u_1_for_sim);
    
    % simulate
    [~, x_sim] = ode45(function_sim, dt_sim(end), x_0,...
        odeset('RelTol',1e-8,'AbsTol',1e-8));

    % extract new x_0
    x_0 = x_sim(end,:)';
else % if model has controller
    % execute controller at dt's of simulation
    for i = 2:length(dt_sim)
        u_1_for_sim = veh_cfg.model_simulation.controller(x_0, u_1);
        function_sim = @(t, x_in_sim) veh_cfg.model_simulation.ode(x_in_sim, u_1_for_sim);

        % simulate
        % NOTE reduced accuracy due to fluctuations of model
        %   to debug:
        %       1. call `figure`
        %       2. expand `odeset` with `'Stats','on','OutputFcn',@odeplot`
        [~, x_sim] = ode45(function_sim, [dt_sim(i - 1) dt_sim(i)], x_0,...
            odeset('RelTol',1e-4,'AbsTol',1e-4));
        
        % extract new x_0
        x_0 = x_sim(end,:)';
    end
end