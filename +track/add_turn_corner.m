
% Track element definition

function checkpoints = add_turn_corner(checkpoints, phi, L, width)
    checkpoints = track.add_turn_base(checkpoints, phi, L, width, 30);
end
