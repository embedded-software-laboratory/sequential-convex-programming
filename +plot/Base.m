classdef Base < handle
    %BASE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        figure_handle
        handles_tmp % handles of temporary plot objects
        is_background_plotted
    end
    
    methods
        function obj = Base(figure_handle_number)
            %BASE Construct an instance of this class
            %   Detailed explanation goes here
            obj.figure_handle = figure(figure_handle_number); % create or get existing figure
            obj.handles_tmp = {};
            obj.is_background_plotted = false;
        end
    end
    methods (Abstract)
        outputArg = plot(obj, scn, ws)
    end
end

