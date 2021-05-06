classdef Linear < model.vehicle.Base
    % Linear vehicle model
    % Controls:
    %   a_x acceleration longitudinal
    %   a_y acceleration lateral
    
    properties
        Ad
        Bd
    end
    
    methods
        function obj = Linear(p)
            obj@model.vehicle.Base(p, 4, 2) % call superclass constructor
            
            dt = obj.p.dt;
            ddt = dt * dt / 2;
            
            %% Linear model: x1 = Ax + Bu
            obj.Ad = [
                1 0 dt 0;
                0 1 0 dt;
                0 0 1 0;
                0 0 0 1];
            obj.Bd = [
                ddt 0;
                0 ddt;
                dt 0;
                0 dt];
        end
            
        function dx = ode(obj, x, u)
            % TODO - is it correct? at least needs matching dt to obj.p.dt
            % for simulation
            dx = (obj.Ad - eye(size(obj.Ad))) * x + obj.Bd * u;
        end
        
        function [Ad, Bd, Ed] = calculatePredictionMatrices(obj, ~, ~)
            % as model is linear, linearization doesn't depend on working
            % point. Thus, additional arguments not required
            
            % expand linear model to all prediction steps
            Ad = repmat(obj.Ad, 1, 1, obj.p.Hp);
            Bd = repmat(obj.Bd, 1, 1, obj.p.Hp);
            Ed = repmat(zeros(obj.nx, 1), 1, 1, obj.p.Hp);
        end
    end
end