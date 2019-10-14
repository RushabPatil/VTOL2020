%% AE 4723
clc;clear all;close all;
% Steady_Climb
% With given aircraft properties, determine speed which maximizes rate of
% climb at sea level
alt = 0;
[~,rho] = Std_Atm_Model(alt,0);

% define aircraft properties (Start with Gulfstream IV)
W = 73000;      % lbf
S = 950;        % ft^2
Cd0 = 0.015;    % unitless
AR = 5.92;      % unitless
e = 0.9;        % unitless
K = 0.08;       % unitless
T = 13000;      % lbf

% Velocity range
v_range = 200:100:1000; % ft/s
v_length = length(v_range);
% calculate aircraft performance ratios
T2W = T/W;      % thrust to weight, unitless
W2S = W/S;      % wetted area, lbf/ft^2
L2D_max = 1/sqrt(4*Cd0*K);

% use small angle approximation

for i = 1:v_length
    v_i = v_range(i);
    RC(i) = v_i*(T2W - (rho*v_i^2*Cd0)/(2*W2S) - (2*W2S*K)/(rho*v_i^2));
end
plot(v_range,RC,v_range,zeros(1,v_length),'r--');
title('Rate of Climb of Gulfstream IV at Sea Level');
xlabel('Airspeed Velocity (ft/s)');ylabel('Rate of Climb (ft/sec)');
% find maximum rate of climb
z = 1 + sqrt(1 + 3/(L2D_max^2*T2W^2));
RC_max = sqrt(W2S*z/(3*rho*Cd0))*T2W^(3/2)*(1 - z/6 - 3/(2*T2W^2*L2D_max^2*z));
disp("Maximum Rate of Climb at Sea Level: " + num2str(RC_max) + "ft/s")