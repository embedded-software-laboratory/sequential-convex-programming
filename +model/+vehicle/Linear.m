classdef Linear < model.vehicle.Base
    % Linear vehicle model
    % Controls:
    %   a_x acceleration longitudinal
    %   a_y acceleration lateral
    
    properties
        Ad
        Bd
    end
    
    methods (Static)
        function p = getParamsCarMaker(dt)
            % CarMaker model from B. Alrifaee and J. Maczijewski, ‘Real-time Trajectory optimization for Autonomous Vehicle Racing using Sequential Linearization’, in 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, Jun. 2018, pp. 476–483, doi: 10.1109/IVS.2018.8500634.
            % Linear model: x1 = Ax + Bu
            ddt = dt * dt / 2;
            p.Ad = [
                1 0 dt 0;
                0 1 0 dt;
                0 0 1 0;
                0 0 0 1];
            p.Bd = [
                ddt 0;
                0 ddt;
                dt 0;
                0 dt];
        end
    end
    
    methods
        function obj = Linear(Hp, dt, p)
            obj@model.vehicle.Base(4, 2, Hp, dt, p) % call superclass constructor
        end
            
        function dx = ode(obj, x, u)
            % TODO - is it correct? at least needs matching dt to obj.p.dt
            % for simulation
            dx = (obj.p.Ad - eye(size(obj.p.Ad))) * x + obj.p.Bd * u;
        end
        
        function [Ad, Bd, Ed] = calculatePredictionMatrices(obj, ~, ~)
            % as model is linear, linearization doesn't depend on working
            % point. Thus, additional arguments not required
            
            % expand linear model to all prediction steps
            Ad = repmat(obj.p.Ad, 1, 1, obj.Hp);
            Bd = repmat(obj.p.Bd, 1, 1, obj.Hp);
            Ed = repmat(zeros(obj.nx, 1), 1, 1, obj.Hp);
        end
    end
end