%% AE 4723 Hw2
clc;clear all;close all;

% Flight properties
altitude = 30000;   % cruise altitude, ft
metric = 0; % not dealing w/ metric units (input to standard atmospheric model)
% Calculate temperature and density of air at cruise altitude via the
% standard atmospheric model
[T,rho] = Std_Atm_Model(altitude,0);
V = 675;    % airspeed relative velocity, ft/s

% Calculate mach number for flight at given cruise speed and altitude
gamma = 1.4;    % adiabatic index of a diatomic gas (air)
R = 8.3145;     % universal gas constant
M = 28.97*10^-3;      % Molar mass of air, kg/mol
speed_of_sound = sqrt(gamma*R*T/M); % Speed of sound, m/s
speed_of_sound_imp = speed_of_sound*3.2808; % Speed of sound, m/s
Mach = V/speed_of_sound_imp;    % Mach number of aircraft at cruise

% wing properties
b = 53.33;       % wingspan, ft
root = 8.2;     % wing root chord length, ft
tip = 2.5;      % wing tip chord length, ft
c = 5.86;   % mean wing chord length, ft
lambda = tip/root;  % wing taper ratio
S = c*b; % wetted wing area, ft^2
AR = b^2/S;     % wing aspect ratio
Sweep = deg2rad(4); % wing sweep angle, radians

% AR is greater than 4, so use the following formula for k (part of
%lift-curve slope)
k_w = 1 + (8.2 - 2.3*Sweep - AR*(0.22 - 0.153*Sweep))/100;
% find wing lift-curve slope
a_w = (2*pi*AR)/(2 + sqrt((AR/k_w)^2*(1 - Mach^2)*(1 + tan(Sweep)^2/(1 - Mach^2)) + 4));

% tail properties
b_t = 20.83;    % tail span, ft
root_t = 4.7;   % tail root chord length, ft
tip_t = 2;      % tail tip chord length, ft
c_t = 0.5*(root_t + tip_t); % mean tail chord length, ft
S_t = c_t*b_t;   % wetted tail area, ft^2
AR_t = b_t^2/S_t;   % tail aspect ratio
Sweep_t = deg2rad(23);   % tail sweep angle, radians
i_t = deg2rad(-3);  % tail incidence angle, radians

% AR_t is greater than 4, so use the following formula for k (part of
%lift-curve slope)
k_t = 1 + (8.2 - 2.3*Sweep_t - AR_t*(0.22 - 0.153*Sweep_t))/100;
% find tail lift-curve slope
a_t = (2*pi*AR_t)/(2 + sqrt((AR_t/k_t)^2*(1 - Mach^2)*(1 + tan(Sweep_t)^2/(1 - Mach^2)) + 4));

% other aircraft propreties
l_tv = 8.8;   % vertical distance between airfoil aerodynamic centers, ft
W = 13000;  % aircraft weight, lbf
Cm_ac = -0.01;  % moment coefficient at wing aerodynamic center
Cd0 = 0.02; % drag constant in tail drag polar
K = 0.062;  % constant in tail drag polar
h_ac = 0.25;    % distance to wing aerodynamic center from leading edge of wing
a_e = 0.5;  % elevator effectiveness
l_t = 23.3-h_ac*c + 0.25*c_t;   % horizontal distance between airfoil aerodynamic centers, ft

% Find dEp/dAlpha
dEp_dAlpha = 4.44*sqrt(1-Mach^2)*(((1/AR) - (1/(1 + AR^1.7)))*((10-3*lambda)/7)*((1 - (l_tv/b))/(2*l_t/b)^0.33)*sqrt(cos(Sweep)))^1.19;
% find location of neutral point
% Neutral point extending small angle approximation (alpha^2 = 0)
% setup tail volume ratios for both horizontal and vertical distances
% between aerodynamic centers
VH = (S_t/S)*(l_t/c);
VH_t = (S_t/S)*(l_tv/c);

% neutral point is a sum of a fraction with the location of the wing
% aerodynamic center
num_simp = a_t*(1 - dEp_dAlpha)*(VH - VH_t*((K*a_t - 1)*2*i_t));
den_simp = a_w + a_t*(1 - dEp_dAlpha)*(S_t/S);
h_NP = h_ac + (num_simp/den_simp)

stat_margin = h_NP - h_ac

%% 2): 
% Plot Moment Coefficient at the Center of Gravity against Angle of Attack
% for different elevator angles (h = 0.25)

del_es = -15:5:15;      % range of delta_e values, degrees
Alphas = -5:0.001:10;   % range of angles of attack, degrees
Als = deg2rad(Alphas);  % range of angles of attack, radians
% determine moment coefficient for different alphas and delta e values
Cm_0 = Cm_ac + VH_t*Cd0;  % constant in Cm
Cm_at = -VH*a_t;   % dependence of Cm on alpha_t
Cm_at2 = VH_t*(K*a_t^2 - a_t);  % dependence of Cm on alpha_t^2
Cm_e = -VH*a_e;  % dependence of Cm on delta_e
Cm_e2 = VH_t*K*a_e^2;   % dependence of Cm on delta_e^2
Cm_ae = VH_t*(2*K - 1)*(a_t*a_e);  % dependence of Cm on alpha*delta_e

for i = 1:numel(del_es)
    % select delta_e for current set of data
    del_e = deg2rad(del_es(i));
    % calculate Cm for range of Alphas
    for a = 1:numel(Alphas)
        % extract angle of attack in radians
        Al = Als(a);
        % generate respective tail AoA
        Al_t = Al*(1 - dEp_dAlpha) + i_t;
        % Approximate alpha_t^2 (assuming alpha^2 = 0)
        Al_t2 = i_t^2 + 2*i_t*Al*(1 - dEp_dAlpha);
        % calculate moment coefficient
        Cm(i,a) = Cm_0 + Cm_at*Al_t + Cm_at2*Al_t2 + del_e*(Cm_e + Cm_e2*del_e + Cm_ae*Al_t);
        % Cm is a matrix of data where a given value Cm_ij has the ith
        % elevator angle and the jth angle of attack
    end
end
% plot data
figure
plot(Alphas,Cm,Alphas,zeros(1,numel(Alphas)),'--k')
legend('Del_e = -15','Del_e = -10','Del_e = -5','Del_e = 0','Del_e = 5','Del_e = 10','Del_e = 15','Cm = 0')
xlabel('Angle of Attack (Degrees)');ylabel('Moment Coefficient at CG');
title('Dependence of Moment Coefficient on Elevator Angle')

%% find trim angles of attack for each given elevator angle
Cm_abs = abs(Cm);
% find moment coefficients closest to zero (minmimum absolute values)
[trims, trim_ind] = min(Cm_abs,[],2);
trim_angles = Alphas(trim_ind)
%% 3):
% elevator angle for trim at given velocity and altitude

% first, determine angle of attack required to maintain trim by force
% balance

% L = W; or CL_net = W/(Q*S)
% Q: dynamic pressure, slug/ft^2
Q = 0.5*rho*V^2;

% find "Average" lift coefficient needed to maintain steady-level flight at
% trim
CL_req = W/(Q*S);

% "Average" Coefficient of lift is a weighted average of the wing and tail
% coefficients of lift
%Cl_avg = x(1)*(a_w + (S_t/S)*a_t*(1 - dEp_dA)) + (S_t/S)*(i_t + a_e*x(2));

% find when a weighted average of Coefficient of lifts equals "CL_req" (or
% more accurately, when the difference between these two variables is zero
fun_Cl_dif = @(x)CL_req - (x(1)*(a_w + (S_t/S)*a_t*(1 - dEp_dAlpha)) + (S_t/S)*(a_t*i_t + a_e*x(2)));

% trim also requires the moment coefficient to be 0
fun_Cm = @(x)Cm_0 + Cm_at*(x(1)*(1 - dEp_dAlpha) + i_t) + Cm_at2*(2*i_t*(1 - dEp_dAlpha)*x(1) + i_t^2) + x(2)*(Cm_e + Cm_e2*x(2) + Cm_ae*(x(1)*(1 - dEp_dAlpha) + i_t));

% initial guess of alpha and delta_e
x0 = zeros(2,1);

% find angles to minimize functions
[angles, optimum] = fminsearch(@(x)fun_Cl_dif(x)^2 + fun_Cm(x)^2, x0);

% convert to degrees
alpha_req = rad2deg(angles(1))
de_req = rad2deg(angles(2))

%%
alpha_t = angles(1)*(1 - dEp_dAlpha) + i_t
alpha_t*a_t*(S_t/S)
a_e*angles(2)*S_t/S