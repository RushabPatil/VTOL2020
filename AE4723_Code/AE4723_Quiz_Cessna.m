%% AE4723_Hw3
clc;clear all;close all;

%% Leerjet 24 In-class Example 11/28
% Input Problem Data
S = 174;        % ft^2
m = 2650;      % aircraft mass, lb
c_bar = 4.9;      % mean wing chord, ft
b = 36;         % wing span, ft
X_CG = 0.264;    % CG location behind leading edge percent of chord length
Ix = 948;     % x-axis MoI, slug ft^2
Iy = 1346;     % y-axis MoI, slug ft^2
Iz = 1967;     % z-axis MoI, slug ft^2
Ixz = 0;     % xz-plane MoI, slug ft^2
g = 32.2;      % gravitational acceleration, ft/s
% Trim conditions
alt = 5000;    % cruise altitude, ft
theta0 = 0; % deg
u0 = 220.1;   % cruise speed, ft/s
q_bar = 49.6;  % dynamic pressure, lbf/ft^2
Mach = 0.201; % mach number of cruise
[~,rho] = Std_Atm_Model(alt,0);
a1 = 0;   % trim angle of attack, degrees
% Aero Data Sheet
% Table B6.4
% Steady State
cl1 = 0.307;
cd1 = 0.032; % Drag at trim, "Cd0" makes more sense
cm1 = 0;
ctx1 = 0.032;
cmt1 = 0;
% Stability Derivatives
cd0 = 0.027;
cdu = 0;
cda = 0.121;
ctxu = -0.096;
cl0 = 0.307;
clu = 0;
cla = 4.41;
cla_dot = 1.7;
clq = 3.9;
cm0 = 0.04;
cmu = 0;
cma = -0.613;
cma_dot = -7.27;
cmq = -12.4;
cmtu = 0;
cmta = 0;
% Control Derivatives
cd_dele = 0;
cd_ih = 0.43;
clde_cdih = cd_dele/cd_ih;
cl_dele = 0.46;
cl_ih = 0.94;
clde_clih = cl_dele/cl_ih;
cm_dele = -1.24;
cm_ih = -2.5;
cmde_cmih = cm_dele/cm_ih;
% Table B6.7
% Stability Derivatives
clb = -0.11;
clp = -0.45;
clr = 0.16;
cyb = -0.73;
cyp = 0;
cyr = 0.4;
cnb = 0.127;
cntb = 0;
cnp = -0.008;
cnr = -0.2;
% Control Derivatives
cl_delA = 0.178;
cl_delR = 0.019;
cy_delA = 0;
cy_delR = 0.14;
cn_delA = -0.02;
cn_delR = -0.074;
% Nondimensional stability and control derivatives
Cxu = - (cdu + 2*cd1);
Cxa = -(cda - cl1);
Cxa_dot = 0;
Cxq = 0;
Cx_dele = -cd_dele;
Cx_delp = ctxu + 2*ctx1;
Czu = -(clu + 2*cl1);
Cza = -(cla + cd1);
Cza_dot = -cla_dot;
Czq = -clq;
Cz_dele = -cl_dele;
Cz_delp = 0;
Cmu = cmu + 2*cm1;
Cma = cma;
Cma_dot = cma_dot;
Cmq = cmq;
Cm_dele = cm_dele;
Cm_delp = cmtu + 2* cmt1;
% Other constants
Cw0 = m / (0.5*rho*u0^2*S); % Non-dimensional weight
% Longitudinal Dimensional Derivatives
Xu = rho*u0*S*Cw0*sind(theta0) + 0.5*rho*u0*S*Cxu;
Xw = 0.5*rho*u0*S*Cxa;
Xq = 0.25*rho*u0*S*c_bar*Cxq;
Xw_dot = 0.25*rho*c_bar*S*Cxa_dot;
Zu = -rho*u0*S*Cw0*cosd(theta0) + 0.5*rho*u0*S*Czu;
Zw = 0.5*rho*u0*S*Cza;
Zq = 0.25*rho*u0*c_bar*S*Czq;
Zw_dot = 0.25*rho*c_bar*S*Cza_dot;
Mu = 0.5*rho*u0*c_bar*S*Cmu;
Mw = 0.5*rho*u0*c_bar*S*Cma;
Mq = 0.25*rho*u0*c_bar^2*S*Cmq;
Mw_dot = 0.25*rho*c_bar^2*S*Cma_dot;
% Lateral Dimensional Derivatives
Yv = 0.5*rho*u0*S*cyb;
Yp = 0.25*rho*u0*b*S*cyp;
Yr = 0.25*rho*u0*b*S*cyr;
Lv = 0.5*rho*u0*b*S*clb;
Lp = 0.25*rho*u0*b^2*S*clp;
Lr = 0.25*rho*u0*b^2*S*clr;
Nv = 0.5*rho*u0*b*S*cnb;
Np = 0.25*rho*u0*b^2*S*cnp;
Nr = 0.25*rho*u0*b^2*S*cnr;

% Longitudinal Linear Model
Along1 = [Xu/m Xw/m 0 -g*cosd(theta0)];
Along2 = [Zu Zw Zq+m*u0 -m*g*sind(theta0)]/(m-Zw_dot);
Along3 =    [Mu+(Mw_dot*Zu)/(m-Zw_dot), Mw+(Mw_dot*Zw)/(m-Zw_dot),...
            Mq+(Mw_dot*(Zq+m*u0))/(m-Zw_dot), -(Mw_dot*m*g*sind(theta0))/(m-Zw_dot)]/Iy;
Along4 = [0 0 1 0];
Along = [Along1; Along2; Along3; Along4]
eig(Along)
% Lateral Linear Model
Ix_prime = (Ix*Iz-Ixz^2)/Iz;
Iz_prime = (Ix*Iz-Ixz^2)/Ix;
Ixz_prime = Ixz/(Ix*Iz-Ixz^2);

Alat1 = [Yv/m, Yp/m, (Yr/m)-u0, g*cosd(theta0)];
Alat2 = [(Lv/Ix_prime)+Ixz_prime*Nv, (Lp/Ix_prime)+Ixz_prime*Np, (Lr/Ix_prime)+Nr*Ixz_prime, 0];
Alat3 = [Ixz_prime*Lv+(Nv/Iz_prime), Ixz_prime*Lp+(Np/Iz_prime), Ixz_prime*Lr+(Nr/Iz_prime), 0];
Alat4 = [0 1 tand(theta0) 0];
Alat = [Alat1; Alat2; Alat3; Alat4]
% Dimensional Control Derivatives
QS = 0.5*rho*u0^2*S;
X_dele = Cx_dele*QS;
X_delp = Cx_delp*QS;
Z_dele = Cz_dele*QS;
Z_delp = Cz_delp*QS;
M_dele = Cm_dele*QS*c_bar;
M_delp = Cm_delp*QS*c_bar;
Y_dela = cy_delA*QS;
Y_delr = cy_delR*QS;
L_dela = cl_delA*QS*b;
L_delr = cl_delR*QS*b;
N_dela = cn_delA*QS*b;
N_delr = cn_delR*QS*b;

Blong = [X_dele/m, X_delp/m; Z_dele/(m-Zw_dot), Z_delp/(m-Zw_dot);
        M_dele/Iy+Mw_dot*Z_dele/(Iy*(m-Zw_dot)), M_delp/Iy+Mw_dot*Z_delp/(Iy*(m-Zw_dot));
        0, 0];
Blat =  [Y_dela/m, Y_delr/m; L_dela/Ix_prime+Ixz_prime*N_dela, L_delr/Ix_prime+Ixz_prime*N_delr;
        N_dela/Iz_prime+Ixz_prime*L_dela, N_delr/Iz_prime+Ixz_prime*L_delr;
        0, 0];    
