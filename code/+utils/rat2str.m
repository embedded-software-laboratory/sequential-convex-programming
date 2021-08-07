function str = rat2str(rat_)
    % create string version of rational
    str = regexprep(rats(rat_), '\s+', '');
end