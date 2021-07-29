function [checkpoints, creation_scale] = testCircuitLiniger
    % scale 1:43
    % by Botz
    % track similar to A. Liniger, A. Domahidi, and M. Morari, “Optimization-Based Autonomous Racing of 1:43 Scale RC Cars,” Optim. Control Appl. Meth., vol. 36, no. 5, pp. 628–647, Sep. 2015, doi: 10.1002/oca.2123.
    creation_scale = 1/43;

    trackWidth = 0.3;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];
    checkpoints.ds = 0;

    checkpoints = model.track.add_turn_N50(checkpoints, 0, 1.25, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, -0.25, 1.5, trackWidth);
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 1, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, -0.25, 0.8, trackWidth);
    
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.4, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, 0.25, 0.4, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, -0.25, 0.4, trackWidth);
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.6, trackWidth);
    
    checkpoints = model.track.add_turn_N30(checkpoints, -0.5, 0.6, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, 0.5, 0.6, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, -0.25, 0.3, trackWidth);
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.4, trackWidth);
    
    checkpoints = model.track.add_turn_N30(checkpoints, -0.375, 0.525, trackWidth);
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.6, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, 0.375, 1.3, trackWidth);
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.6, trackWidth);
    
    checkpoints = model.track.add_turn_N30(checkpoints, 0.25, 0.6, trackWidth);
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.3, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, 0.375, 0.525, trackWidth);
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 0.5, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, -0.5, 0.6, trackWidth);
    
    checkpoints = model.track.add_turn_N50(checkpoints, 0, 1.6, trackWidth);
    checkpoints = model.track.add_turn_N30(checkpoints, -0.375, 0.525, trackWidth);
    
    checkpoints = checkpoints(2:end); % select checkpoints 2 till end
end