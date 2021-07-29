function vector_global = vector_local2global(vector_local, yaw)
    % convert global to local (vehicle) reference frame
    %
    % input and output vertical vector
    %
    % global
    %   vector_x
    %   vector_y
    % local
    %   vector_long
    %   vector_lat
    %   yaw
    vector_global = [cos(yaw), -sin(yaw); 
                     sin(yaw),  cos(yaw)] * vector_local;