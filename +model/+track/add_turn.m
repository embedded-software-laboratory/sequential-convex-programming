
% Track element definition

function checkpoints = add_turn(checkpoints, phi, L, width)
    checkpoints = model.track.add_turn_base(checkpoints, phi, L, width, 40);
end
