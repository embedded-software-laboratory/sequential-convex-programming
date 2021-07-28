% this package contains all configurations

%% Config Types
% there are 3 config types:
% - config
%       overall settings
% - scenarios
%       adds scenario specific settings to config, such as track, vehicles,
%       obstacles, etc.
% - vehicles
%       are added in a scenario config, set vehicle-specific settings such
%       as model, weights, start position, etc.
%
%       Naming Convention for models & parameters
%           'vehicle_<model>_<parameters>
%           if not the same models and parameters are used for controller & simulation, it expands to
%           'vehicle_<controller model>_<controller parameters>_<simulation model>_<simulation parameters>'

%% How to Use
% each type has a default config, 'scenario' and 'vehicle'. This ensures shared parameters.

% To change certain settings, you can use the existing configuration files 
% by cascading them, e.g. for a vehicle
%           1. getting default vehicle parameters
%       2. change vehicle model
%   3. change track discreitization to SCR
vehicle_config = ...
    config.vehicle_SCR(...
        config.vehicle_ST_Liniger(...
            config.base_vehicle()));
