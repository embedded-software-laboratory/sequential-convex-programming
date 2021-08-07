% Track element definition

function checkpoints = add_turn_N40(checkpoints, phi, L, width)
    checkpoints = model.track.add_turn(checkpoints, phi, L, width, 40);
end