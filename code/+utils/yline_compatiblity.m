function yline_compatiblity(yvalues, LineSpec)
    % compatibility abstraction for old MATLAB versions
    if verLessThan('matlab', '9.10')
        % seperate eacht data point
        for yvalue = yvalues
            yline(yvalue, LineSpec);
        end
    else
        yline(yvalues, LineSpec);
    end