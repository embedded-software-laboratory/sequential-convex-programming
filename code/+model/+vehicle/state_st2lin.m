function state_lin = state_st2lin(state_st)
    % convert single-track to linear model states
    % single-track
    %   p_x
    %   p_y
    %   v_x
    %   v_y
    %   yaw
    %   dyaw
    % linear
    %   p_x
    %   p_y
    %   v_x
    %   v_y
    state_lin = zeros(4, 1) * NaN;
    state_lin(1:2) = state_st(1:2);
    state_lin(3:4) = model.vehicle.vector_local2global(state_st(3:4), state_st(5));