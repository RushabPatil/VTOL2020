% minimum radius of curvature
clc;clear all;close all;

Cl_max = 1.4;
W = 8375;
T = 1200;
S = 244;
St = 31;
V = 300;
g = 32.15;
[~,rho] = Std_Atm_Model(0,0)
%S_net = S + St;
Q = 0.5*rho*V^2;

n_alpha = Q*Cl_max/(W/S)
Rmin = V^2/(g*sqrt(n_alpha^2 - 1))