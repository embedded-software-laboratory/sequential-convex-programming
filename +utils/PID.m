function [u_out, I, error] = PID(K_p, K_i, K_d, dt, u_ref, u_actual, I, error_prev)
    % stateless PID controller in parallel form
    
    error = u_ref - u_actual;

    % integrate & differntiate
    I = I + error * dt;
    derivative = (error - error_prev) / dt;

    % PID Equation
    u_out = K_p * error + K_i * I + K_d * derivative;
end