%% AE 4723 Steady Level Climb
clc;clear all;close all;
%% Trim: Moment balance about the CG

% Assumptions:
% Small angle approximation for angle of attack (alpha)
% high lift-to-drag ratio (L/D ~ 10; L >> D*alpha)
% can approximate lift curves as linear below stall angle
% can approximate downwash angle as varying linear with angle of attack

% inputs (properties of aircraft and airfoils)
c = 6;          % mean chord length, ft
S = 290;        % wing area, ft^2
S_t = 70;       % tail wing area, ft^2
l_t = 20;       % distance between aerodynamic centers, ft
W = 13000;      % aircraft weight, lbf

a = 6.5;        % wing lift curve slope, radians^-1
a_t = 5.3;      % tail lift curve slope, radians^-1
a_e = 0.5;      % elevator lift effectiveness, radians^-1
dEp_dAlpha = 0.177;     % change of downwash angle with respect to angle of attack
Ep_0 = 0;       % downwash angle at zero angle of attack, radians
Cm_ac = -0.01;  % moment coefficient about wing aerodynamic center
CL0_W = 0;      % wing coefficient of lift at angle of attack = 0
h_ac = 0.25;    % location of wing aerodynamic center (fraction of wing chord)
h = 0.5;        % location of CG, fraction of wing mean chord
i_t = deg2rad(-3);  % incidence angle of tail, radians

% inputs continued (properties of flight)
altitude = 30000;   % altitude above sea level, ft
V = 675; % airspeed velocity, ft/s
metric = 0;         % not metric (applies to all units)

% find air density at given altitude
[~,rho] = Std_Atm_Model(altitude,metric);
Q = 0.5*rho*V^2;    % dynamic pressure, lbf/ft^2

% Components of CL_total
% CL_total = CL0 + CL_Alpha*alpha + CL_Deltae*Deltae

CL_0 = CL0_W + (S_t/S)*a_t*(i_t - Ep_0);
CL_Deltae = (S_t/S)*a_e;
CL_Alpha = a_t*(1 - dEp_dAlpha)*(S_t/S) + a;
CL_total = W/(Q*S);

V_H = (S_t/S)*(l_t/c);      % horizontal tail volume ratio

% Components of Cm
% Cm = Cm_0 + Cm_Alpha*alpha + Cm_Deltae*Deltae
Cm_0 = Cm_ac + CL0_W*(h - h_ac) - a_t*(i_t - Ep_0)*V_H*(1 - (h - h_ac)*(c/l_t));
Cm_Alpha = (a + a_t*(S_t/S)*(1 - dEp_dAlpha))*(h - h_ac) - a_t*V_H*(1 - dEp_dAlpha);
Cm_Deltae = CL_Deltae*(h - h_ac) - a_e*V_H;

% Setup linear algebra problem
A = [CL_Alpha CL_Deltae; Cm_Alpha Cm_Deltae];
B = [CL_total - CL_0; -Cm_0];

x = A\B
Alpha = x(1);   % wing angle of attack at trim (radians)
Deltae = x(2);  % elevator angle at trim (radians)
% Convert to degrees
Alpha_d = rad2deg(Alpha);
Deltae_d = rad2deg(Deltae);
% Check that moment coefficient is zero about CG
Cm = Cm_0 + Cm_Alpha*Alpha + Cm_Deltae*Deltae
% Check that moment is zero about CG
M = 0.5*rho*V^2*S*c*Cm

% location of neutral point (might need to redo?)
hNP = (a_t/CL_Alpha)*(1 - dEp_dAlpha)*V_H + h_ac;  % location of neutral point (fraction of wing mean chord)
