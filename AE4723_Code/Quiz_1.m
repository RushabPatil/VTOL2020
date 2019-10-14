%% AE 4723 quiz 1
clc;clear all;close all;
% BD-5J

S = 3.512; % m^2
W = 4270; % N
T = 898.5; % N
Cd0 = 0.02;
K = 0.062;

metric = 1;
rho_sl = 1.225; % kg/m^3

%% A

L_D_max = 1/sqrt(4*Cd0*K)

V_range = 40:20:80; % m/s

alt = 1000; % m
% determine properties for all 3 different altitudes
[L_to_D,Thrusts] = Aircraft_Performance(alt,Cd0,W,K,S,V_range,metric)

%% B

alt = 0; % m
RCmax = Rate_of_Climb(W,S,Cd0,T,K,alt,metric)
T2W = T/W;
W2S = W/S;

z = 1 + sqrt(1 + 3/(L_D_max*T2W)^2);
v_RCmax = sqrt(T2W*W2S*z/(3*rho_sl*Cd0))
gamma = asind(RCmax/v_RCmax)

%% C

alts = [3000 5000 8000];
for i = 1:3
    alt_i = alts(i);
    RCmaxs(i) = Rate_of_Climb(W,S,Cd0,T,K,alt_i,metric);
end
disp(RCmaxs)

rc_ceiling = 100/(60 *3.2808)
alt_check = 9000:500:11000
for i = 1:5
    alt_i = alt_check(i);
    RCchecks(i) = Rate_of_Climb(W,S,Cd0,T,K,alt_i,metric);
end
disp(RCchecks)