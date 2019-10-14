%% Level turning (Sea Level)
clc;clear all;close all;
v_range = 200:100:800;
rho = 2.377e-3;
L2D = 14.46;
T2W = 0.178;
W2S = 76.84;
Clmax = 1.2
Cd0 = 0.015;
K = 0.0797;
for i = 1:numel(v_range)
v_i = v_range(i);
% nmax,alpha
Q_i = 0.5*rho*v_i^2;
n_alpha(i) = Q_i*Clmax/(W2S);

n_T(i) = sqrt((Q_i/(K*W2S))*(T2W - (Cd0*Q_i/W2S)));
end
