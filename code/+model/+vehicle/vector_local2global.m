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
    vector_global = vector_local;
    for i = 1:numel(yaw)
        vector_global(:,i) = [cos(yaw(i)), -sin(yaw(i)); 
                            sin(yaw(i)),  cos(yaw(i))] * vector_local(:,1);
    end