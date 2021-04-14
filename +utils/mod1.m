function y = mod1(i, N)
% Modulo for one-based indices: offsetting modulo to work with Matlab's
% indexing (start is 1, not 0)
y = mod(i-1, N) + 1;
end