function pis = removePiPeriodicity(pis)
% removes periodicity of given vector of pis and moves into range [-pi, pi]

% remove periodicity
pis = mod(pis, 2*pi);
 % move into range [-pi, pi]
pis(pis > pi) = pis(pis > pi) - 2*pi;