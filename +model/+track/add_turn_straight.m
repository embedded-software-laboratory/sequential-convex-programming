
% Track element definition

function checkpoints = add_turn_straight(checkpoints, phi, L, width)
    checkpoints = model.track.add_turn_base(checkpoints, phi, L, width, 50);
end
