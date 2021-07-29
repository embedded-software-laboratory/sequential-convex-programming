% Track element definition

function checkpoints = add_turn(checkpoints, phi, L, width, N)
% width -   set to trackwidth normally
% phi -     segment of circle from 0.0 (straight) to 1.0 (full circle) and
%           direction of turn (- for right turn/clockwise, + for left
%           turn/anti-clockwise)
% L -       arc length of section
% N number of segments for track element

    kappa = (phi*(2*pi))/L;                
    ds = L / N;
    
    for i = 1:N
        checkpoints(end+1).yaw = checkpoints(end).yaw + kappa * ds;
        c = cos(checkpoints(end).yaw);
        s = sin(checkpoints(end).yaw);
        % vector tangential to movement direction (pointing "forward"), normalized
        f = [c;s];
        % normal vector, pointing to the left (called "n" in paper)
        n = [0 -1;1 0] * f;
        
        % save discretization in struct
        checkpoints(end).forward_vector = f;
        checkpoints(end).normal_vector = n;
        checkpoints(end).center = checkpoints(end-1).center + f * ds;
        checkpoints(end).left = checkpoints(end).center + n * width/2;
        checkpoints(end).right = checkpoints(end).center - n * width/2;
        checkpoints(end).ds = ds;
    end
end