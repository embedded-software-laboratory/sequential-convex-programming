function velocity_local = velocity_global2local(velocity_global)
    % convert global velocity to local (vehicle) reference frame
    % transformation assumes zero side slip angle
    %
    % input and output vertical vector
    %
    % global
    %   velocity_x
    %   velocity_y
    % local
    %   velocity_long
    %   velocity_lat
    v_x = velocity_global(1);
    v_y = velocity_global(2);
    velocity_local = 1/sqrt(v_x^2 + v_y^2 + eps) .* [v_x v_y; -v_y v_x] * [v_x; v_y];