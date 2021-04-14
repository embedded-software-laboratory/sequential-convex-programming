classdef Base
    %BASE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        figure_handle
    end
    
    methods
        function obj = Base(figure_handle_number)
            %BASE Construct an instance of this class
            %   Detailed explanation goes here
            obj.figure_handle = figure(figure_handle_number); % create or get existing figure
        end
    end
    methods (Abstract)
        outputArg = plot(obj, scn, ws)
    end
end

