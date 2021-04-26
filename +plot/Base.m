classdef Base < handle
    %BASE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        figure_handle
        handles_tmp % handles of temporary plot objects
        is_background_plotted
        subplot_handles
    end
    
    methods
        function obj = Base(figure_handle_number)
            %BASE Construct an instance of this class
            %   Detailed explanation goes here
            obj.figure_handle = figure(figure_handle_number); % create or get existing figure
            % sometimes (?) helps with performance:
            %set(figure_handle_number,
            %    'MenuBar', 'figure',
            %    'ToolBar', 'figure');
            
            obj.handles_tmp = {};
            obj.is_background_plotted = false;
            obj.subplot_handles = {};
        end
        
        function add_tmp_handle(obj, handle)
            obj.handles_tmp{end + 1} = handle;
        end
            
        
        function clear_tmp(obj)
             % clear old plot objects
            for i = 1:length(obj.handles_tmp)
                delete(obj.handles_tmp{i})
            end
            obj.handles_tmp = {};
        end
    end
    methods (Abstract)
        outputArg = plot(obj, scn, ws)
    end
end

