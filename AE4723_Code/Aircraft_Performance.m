function [Lift_to_Drags,Thrusts] = Aircraft_Performance(altitude,Cd0,Weight,K,S,Velocities,metric)
% Aircraft_Performance takes inputs from a "Benevolent Aerodynamicist" and
% other inputs to find the optimal properties of steady, level flight

% INPUTS:
% altitude          - height above sea level (ft OR m)
% Cd0               - coefficient of non-induced drag (unitless)
% Weight            - weight of plane (ft OR m)
% K                 - weight of coefficient of lift in thee drag polar
%                   equation (unitless)
% S                 - Surface area of wings (ft^2 OR m^2)
% Velocities        - a column vector of velocities to consider in finding
%                   required lift-to-drag ratios for steady, level flight
%                   (ft/s OR m/s)
% metric            - 0: no (imperial units); 1: yes (metric units)

% OUTPUTS:
% Lift_to_Drags     - row vector of Lift-to-drag ratios corresponding to
%                   "Velocities" input (unitless)
% Thrusts           - Thrusts needed for steady, level flight corresponding
%                   to "Velocities" input (lbf OR N)

% VARIABLES:
% rho               - density of air at given altitude (slug/ft^3 OR
%                   kg/m^3)
% iterations        - length of input "Velocities" (unitless)
% V_n               - entry n of "Velocities" (ft/s OR m/s)
% Cl_n              - Coefficient of lift needed to achieve steady, level
%                   flight at velocity "V_n" (unitless)


% Start by finding density of air at given altitude
[~,rho] = Std_Atm_Model(altitude,metric);

% preallocate for speed
iterations = length(Velocities);
Lift_to_Drags = zeros(iterations,1);
Thrusts = zeros(iterations,1);
% find Lift-to-Drag ratio and thrust needed at each velocity input
for n = 1:iterations
    % save current iteration's velocity
    V_n = Velocities(n);
    % determine corresponding Coeffificent of lift
    Cl_n = 2*Weight/(rho*V_n^2*S);
    % determine Lift-to-Drag ratio
    Lift_to_Drags(n) = Cl_n/(Cd0 + K*Cl_n^2);
    % determine required thrust
    Thrusts(n) = Weight/Lift_to_Drags(n);
end
end

