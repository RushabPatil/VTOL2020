%% AE 4723 Quiz 2 Stuff
clc;clear all;close all;
%% Trim: Moment balance about the CG

% Assumptions:
% Small angle approximation for angle of attack (alpha)
% high lift-to-drag ratio (L/D ~ 10; L >> D*alpha)
% can approximate lift curves as linear below stall angle
% can approximate downwash angle as varying linear with angle of attack

% inputs (properties of aircraft and airfoils)
c = 6.8;  % mean chord length, ft
S = 244;    % wing area, ft^2
S_t = 31;   % tail wing area, ft^2
l_t = 18.16;   % distance between aerodynamic centers, ft

a = 4.62;    % wing lift curve slope
a_t = 4.06;  % tail lift curve slope
dEp_dAlpha = 0.1; % change of downwash angle with respect to angle of attack
Ep_0 = 0;   % downwash angle at zero angle of attack, radians
Cm_ac = -0.03;   % moment coefficient about wing aerodynamic center
CL0_w = 0.1;  % wing coefficient of lift at angle of attack = 0
h_ac = 0.25; % location of wing aerodynamic center (fraction of wing chord)
i_t = deg2rad(-3);    % incidence angle of tail, radians

% location of neutral point
V_H = (S_t/S)*(l_t/c)  % horizontal tail volume ratio
a_bar = a_t*(1 - dEp_dAlpha)*(S_t/S) + a
hNP = (a_t/a_bar)*(1 - dEp_dAlpha)*V_H + h_ac % location of neutral point (fraction of wing mean chord)

%% pitching moment coefficient at CG for given location of CG and given alpha
h = 0.25;    % location of CG, fraction of wing mean chord
alpha = 0;
Cm_0 = Cm_ac + CL0_w*(h - h_ac) - a_t*(i_t - Ep_0)*V_H*(1 - (h - h_ac)*(c/l_t))
dCm_dAlpha = (a + a_t*(S_t/S)*(1 - dEp_dAlpha))*(h - h_ac) - a_t*V_H*(1 - dEp_dAlpha)
Cm = Cm_0 + dCm_dAlpha*alpha

% find net pitching moment on CG
altitude = 3000;   % ft
metric = 0;         % not metric
[~,rho] = Std_Atm_Model(altitude,metric)
V = 300; % ft/s

M = 0.5*rho*V^2*S*c*Cm