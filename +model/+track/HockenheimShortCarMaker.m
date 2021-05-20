function [checkpoints, creation_scale] = HockenheimShortCarMaker(checkpoint_distance, trackWidth)
% inputs e.g. 1, 7
% recorded from CarMaker's Hockenheim race track (see B. Alrifaee and J. Maczijewski, “Real-time Trajectory optimization for Autonomous Vehicle Racing using Sequential Linearization,” in 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483. doi: 10.1109/IVS.2018.8500634.

creation_scale = 1/1;

if ~exist('checkpoint_distance', 'var'); checkpoint_distance = 1; end
if ~exist('trackWidth', 'var'); trackWidth = 7; end

% loads and prepares reald world (?) path of Hockenheim
data = load('model/track/HockenheimShortCarMaker.mat');
path = data.vehicle_path.Data(:,1:2);

checkpoints = struct('left',{},'right',{},'center',{});

%% Arclength parameterization
arclen = [0 cumsum(sqrt(sum(diff(path)'.^2)))];
arclen_new = 0:1:max(arclen);
path = [interp1(arclen, path(:,1), arclen_new);interp1(arclen, path(:,2), arclen_new)]';

%% Discretize
first_point = path(1,:);
last_point = path(1,:);
for i = 2:size(path,1)
    % Remove some checkpoints, they are too dense: if sparse enough
    if norm(path(1,:) - last_point(end,:)) > checkpoint_distance
        if i > size(path,1)/2 && norm(path(1,:) - first_point(1,:)) < 4 % Abort before the loop closes
            break
        end
        last_point = path(1,:);


        %% Create checkpoint struct            
        % vector tangential to movement diraction, normalized
        t = path(2,:)' - path(1,:)';
        t = t / norm(t);

        % normal vector, pointing to the left (called "n" in paper)
        n = [0 -1;1 0] * t;

        % save discretization in struct
        checkpoints(end + 1).forward_vector = t;
        checkpoints(end).normal_vector = n;
        checkpoints(end).center = path(1,:)';
        checkpoints(end).left = path(1,:)' + n * trackWidth/2;
        checkpoints(end).right = path(1,:)' - n * trackWidth/2;
    end
    path = circshift(path,-1);
end
end