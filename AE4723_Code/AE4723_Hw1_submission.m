%% AE 4723 Homework 1
clc;clear all;close all;
%% Problem 1
disp("Problem 1")
% Using Gulfstream IV example from 10_24_2018 lecture, determine properties
% of ideal travel at steady, level flight (imperial units)

% inputs based on plane
S = 950;                    % ft^2
Cd0 = 0.015;                % unitless
W = 73000;                  % lbf
alts = [10000 30000 50000]; % ft
K = 0.0797;                   % unitless

% note we are NOT dealing with metric units
metric = 0;

% range of velocity inputs
V_range = 200:1:1000; % ft/s
V_length = length(V_range);

L_to_D = zeros(V_length,3);
Thrusts = zeros(V_length,3);

% determine properties for all 3 different altitudes
for i = 1:3
    alt = alts(i);
    [L_to_D(:,i),Thrusts(:,i)] = Aircraft_Performance(alt,Cd0,W,K,S,V_range,metric);
end
figure
plot(V_range,L_to_D(:,1),V_range,L_to_D(:,2),'r',V_range,L_to_D(:,3),'g');xlabel('Velocity (feet/sec)');
ylabel('Lift-to-Drag Ratio (unitsless)');
title('Steady Level Flight: Lift to Drag')
legend('10,000 ft', '30,000 ft','50,000 ft','Location','southeast')

% determine maximum lift to drag ratio on given interval of velocities, and
% find corresponging velocity

for i = 1:3
    disp("Altitude: " + num2str(alts(i)) + " ft")
    [val,ind] = max(L_to_D(:,i));
    disp("Maximum Lift-to-drag Ratio: " + num2str(val))
    v_ideal = V_range(ind);
    disp("Velocity of Maximum Lift-to-drag Ratio: " + num2str(v_ideal) + " ft/s")
end
%% Problem 2
disp(" ")
disp("Problem 2")
% plot thrust ranges against airspeed velocity for same altitudes above
figure
plot(V_range,Thrusts(:,1),V_range,Thrusts(:,2),'r',V_range,Thrusts(:,3),'g');
xlabel('Velocity (feet/sec)');ylabel('Thrust (lbf)');
title('Steady Level Flight: Thrust')
legend('10,000 ft', '30,000 ft','50,000 ft','Location','northeast')

% find ideal flight based on velocity range (may be off by a few ft/s due
% to the step size when plotting)
[L_D_max,ind_ideal] = max(L_to_D);
Thrust_min = zeros(1,3);
for col = 1:3
    ind_col = ind_ideal(col);
    Thrust_min(col) = Thrusts(ind_col,col);
end
V_ideal = V_range(ind_ideal);
disp("Cruise altitudes: " + num2str(alts) + " ft")
disp("Minimum Thrust: " + num2str(Thrust_min) + " lbf")

%% Problem 3
disp(" ")
disp("Problem 3")
% Given loss of thrust in Gulfstream IV at 30,000 ft, minimize flight path
% angle

% define aircraft properties (Start with Gulfstream IV)
W = 73000;      % lbf
S = 950;        % ft^2
Cd0 = 0.015;    % unitless
AR = 5.92;      % unitless
e = 0.9;        % unitless
K = 0.0797;       % unitless

h = 30000; % ft
v0 = 800; % ft/s
T2W = 0;
W2S = W/S;

% a) find expression and numerical value for flight path angle for steady
% descent

% find air density at given altitude
[~,rho] = Std_Atm_Model(h,0);

% find rate of descent for given airspeed velocity
R_descent = v0*(T2W - (rho*v0^2*Cd0)/(2*W2S) - (2*W2S*K)/(rho*v0^2));

% find corresponding flight path angle given airspeed velocity and rate of
% descent
gamma = asind(R_descent/v0);
disp("Rate of steady descent (V = 800 ft/s): " + num2str(R_descent) + " ft/s")
disp("Flight path angle of steady descent: " + num2str(gamma) + " degrees")
% b) If possible, find numerical value of airspeed to minimize flight path
% angle @ 30,000 ft

% brute force finding the velocity of minimum descent rate

v_values = 200:10:1000;
for i = 1:numel(v_values)
    v0 = v_values(i);
    R_descents(i) = v0*(T2W - (rho*v0^2*Cd0)/(2*W2S) - (2*W2S*K)/(rho*v0^2));
end
figure
plot(v_values,R_descents)
xlabel('Velocity (feet/sec)');ylabel('Rate of Descent (ft/s)');
title('Steady Descent, No Thrust')
% find slowest rate of descent (least negative value)
[val,ind] = max(R_descents);
v_ideal = v_values(ind);
disp("Airspeed velocity of minimum descent rate: " + num2str(v_ideal) + " ft/sec")
disp("Minimum descent rate: " + num2str(val) + " ft/sec")