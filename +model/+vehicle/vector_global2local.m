function vector_local = vector_global2local(vector_global, yaw)
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
    vector_local = [ cos(yaw), sin(yaw);
                    -sin(yaw), cos(yaw)] * vector_global;