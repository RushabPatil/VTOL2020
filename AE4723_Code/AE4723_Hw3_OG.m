%% AE4723_Hw3
clc;clear all;close all;

%% Leerjet 24 In-class Example 11/28
% Input Problem Data
g = 32.174;     % gravitational acceleration, ft/s
S = 230;        % ft^2
m_lbf = 13000;  % aircraft weight, lb
m = m_lbf/g;    % aircraft mass, slug
c_bar = 7;      % mean wing chord, ft
b = 34;         % wing span, ft
X_CG = 0.32;    % CG location behind leading edge percent of chord length
Ix = 28000;     % x-axis MoI, slug ft^2
Iy = 18800;     % y-axis MoI, slug ft^2
Iz = 47000;     % z-axis MoI, slug ft^2
Ixz = 1300;     % xz-plane MoI, slug ft^2
% Trim conditions
alt = 40000;    % cruise altitude, ft
theta0 = 0; % deg
theta0 = deg2rad(theta0);
u0 = 677;   % cruise speed, ft/s
q_bar = 134.6;  % dynamic pressure, lbf/ft^2
Mach = 0.7; % mach number of cruise
[~,rho] = Std_Atm_Model(alt,0);
a1 = 2.7;   % trim angle of attack, degrees
% Aero Data Sheet
% Table B6.4
% Steady State
cl1 = 0.41;
cd1 = 0.0335; % Drag at trim, "Cd0" makes more sense
cm1 = 0;
ctx1 = 0.0335;
cmt1 = 0;
% Stability Derivatives
cd0 = 0.0216;
cdu = 0.104;
cda = 0.3;
ctxu = -0.07;
cl0 = 0.13;
clu = 0.4;
cla = 5.84;
cla_dot = 2.2;
clq = 4.7;
cm0 = 0.05;
cmu = 0.05;
cma = -0.64;
cma_dot = -6.7;
cmq = -15.5;
cmtu = -0.003;
cmta = 0;
% Control Derivatives
cd_dele = 0;
cd_ih = 0;
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
Cw0 = m_lbf / (0.5*rho*u0^2*S); % Non-dimensional weight
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

% Eigenvalues of Along
Long_modes = eig(Along)
% 1) complex pair with small magnitude real part: Phugoid
% 2) complex pair with larger magnitude real part: Short period
% Lateral Linear Model
Ix_prime = (Ix*Iz-Ixz^2)/Iz;
Iz_prime = (Ix*Iz-Ixz^2)/Ix;
Ixz_prime = Ixz/(Ix*Iz-Ixz^2);

Alat1 = [Yv/m, Yp/m, (Yr/m)-u0, g*cosd(theta0)];
Alat2 = [(Lv/Ix_prime)+Ixz_prime*Nv, (Lp/Ix_prime)+Ixz_prime*Np, (Lr/Ix_prime)+Nr*Ixz_prime, 0];
Alat3 = [Ixz_prime*Lv+(Nv/Iz_prime), Ixz_prime*Lp+(Np/Iz_prime), Ixz_prime*Lr+(Nr/Iz_prime), 0];
Alat4 = [0 1 tand(theta0) 0];
Alat = [Alat1; Alat2; Alat3; Alat4]
% Eigenvalues
Lat_modes = eig(Alat)
% Three lateral modes
% 1) Complex conjugate pair: Dutch roll mode
% 2) real: roll convergence (fastest lateral mode)
% 3) other real: spiral mode (really really really slow)
% sometimes, 2) + 3) are instead another complex conjugate pair (lateral
% Phugoid)
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


%% 12/5/2018 Demo
% can show dynamic stability by finding eigenvalues (done previously)

% settling time: ts = 4/abs(Re(eig))
% Phugoid mode:
t_s_Phugoid = abs(4/real(Long_modes(3)))
% Roll convergence mode:
t_s_Roll_convg = abs(4/real(Lat_modes(3)))

% Plot impulse and step response for various disturbances
% Elevator, pitch angle
% Thrust, speed
% Rudder, yaw
% Aileron, roll angle
% note: implicity separating lateral and longitudinal control variables
long_sys = ss(Along,[Blong(:,1)*pi/180 Blong(:,2)],eye(4),zeros(4,2));
lat_sys = ss(Alat,Blat,eye(4),zeros(4,2));

% control inputs (listed in orders of respective B matrices): 
% Lat: aileron (da), rudder (dr) 
% Long: elevator (de), thrust (dp)
% figure
% impulse(long_sys)
% isolate one elevator to pitch angle
long_sys_el2pitch = ss(Along, Blong(:,1)*pi/180,[0 0 0 1], 0);
% C = [0 0 0 1] picks out 4th state variable (pitch angle) for observation
% B multiplied to pretend elevator is in degrees
figure('Name','Elevator-Pitch Angle: Impulse');impulse(long_sys_el2pitch)
figure('Name','Elevator-Pitch Angle: Step');step(long_sys_el2pitch)
% isolate thrust to speed
long_sys_t2speed = ss(Along, Blong(:,2), [1 0 0 0], 0);
figure('Name','Thrust-Speed: Impulse');impulse(long_sys_t2speed)
figure('Name','THrust: Step');step(long_sys_t2speed)
% figure
% impulse(lat_sys)
% isoltate aileron to roll response
lat_sys_a2roll = ss(Alat, Blat(:,1)*pi/180, [0 0 0 1],0);
figure('Name','Aileron-Roll Angle: Impulse');impulse(lat_sys_a2roll)
figure('Name','Aileron-Roll Angle: Step');step(lat_sys_a2roll)
lat_sys_a2rrate = ss(Alat, Blat(:,1)*pi/180, [0 1 0 0],0);
figure('Name','Aileron-Roll Rate: Impulse');impulse(lat_sys_a2rrate)
figure('Name','Aileron-Roll Rate: Step');step(lat_sys_a2rrate)
% isolate rudder to yaw angle response
% add psi_dot to Alat, Blat

Alat_with_yaw = [Alat zeros(4,1); 0 0 sec(theta0) 0 0];
Blat_with_yaw = [Blat; 0 0];

lat_sys_r2yaw = ss(Alat_with_yaw, Blat_with_yaw(:,2)*pi/180, [0 0 0 0 1],0);
figure('Name','Rudder-Yaw: Impulse');impulse(lat_sys_r2yaw)
figure('Name','Rudder-Yaw: Step');step(lat_sys_r2yaw)
% ignore data being noisy, assume we are given all states

%% altitude hold control

% start with previous controller and hope it works
% treat altitude as linear model
Along_with_altitude = [Along zeros(4,1); -sin(theta0) cos(theta0) 0 -u0*cos(theta0) 0];    % 5x5
Blong_with_altitude = [Blong; zeros(1,2)];

K_altitude_hold = lqr(Along_with_altitude,Blong_with_altitude,0.01*eye(5), 10*eye(2), zeros(5,2))

% check how gain works
Along_alt_hold = Along_with_altitude - Blong_with_altitude*K_altitude_hold;
long_sys_alt_hold = ss(Along_with_altitude,Blong_with_altitude,eye(5), zeros(5,2));
figure
impulse(long_sys_alt_hold)

% airspeed disturbance
[t_sim_long, x_long] = ode45(@(t,x) Along_alt_hold*x, [0 400], [10 0 0 0 0]');
% plot control action
control_long = (-K_altitude_hold*x_long')';
figure
title('Longitudinal Altitude Hold Response')
subplot(511)
plot(t_sim_long,x_long(:,1))
xlabel('time (s)'); grid on;
ylabel('\Delta u (ft/s)');

subplot(512)
plot(t_sim_long,x_long(:,2))
xlabel('time (s)'); grid on;
ylabel('\Delta w (ft/s)');

subplot(513)
plot(t_sim_long,x_long(:,3)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\Delta q (\circ/s)');

subplot(514)
plot(t_sim_long,x_long(:,4)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\Delta \theta (\circ)');

subplot(515)
plot(t_sim_long,x_long(:,5))
xlabel('time (s)'); grid on;
ylabel('\Delta z (ft)');

figure
title('Longitudinal Altitude Hold Control')
subplot(211)
plot(t_sim_long,control_long(:,1))
xlabel('time (s)'); grid on;
ylabel('\Delta e (radians)');

subplot(212)
plot(t_sim_long,control_long(:,2))
xlabel('time (s)'); grid on;
ylabel('\Delta t (lb)');
return
%% Stick-fixed Response
% airspeed disturbance
[t_sim_long, x_long] = ode45(@(t,x) Along_with_altitude*x, [0 400], [10 0 0 0 0]');
figure
title('Longitudinal Stick-Fixed Response')
subplot(511)
plot(t_sim_long,x_long(:,1))
xlabel('time (s)'); grid on;
ylabel('\Delta u (ft/s)');

subplot(512)
plot(t_sim_long,x_long(:,2))
xlabel('time (s)'); grid on;
ylabel('\Delta w (ft/s)');

subplot(513)
plot(t_sim_long,x_long(:,3)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\Delta q (\circ/s)');

subplot(514)
plot(t_sim_long,x_long(:,4)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\Delta \theta (\circ)');

subplot(515)
plot(t_sim_long,x_long(:,5))
xlabel('time (s)'); grid on;
ylabel('\Delta z (ft)');
% obtain shorter settling time via controls
%% pole placement
% aka Eigenstructure assignment
% real part related to settling time, imaginary part related to how much
% overshoot is tolerable/damping is needed
desired_ev = [  -0.9972 + 2.6511i; -0.9972 - 2.6511i; -0.05 + 0.1114i; -0.05 - 0.1114i];
desired_settling_time = 4/.05;
% control design boils down to designing right gain matrix Klong
Klong = place(Along,Blong,desired_ev);
% x_dot = (A - B*K)*x;  u = -K*x
Along_cl = Along-Blong*Klong;

%% Controlled (closed-loop) response
%{
[t_sim_long, x_long] = ode45(@(t,x) Along_cl*x, [0 100], [10 0 0 0]');
figure
title('Longitudinal Stick-Fixed Response')
subplot(411)
plot(t_sim_long,x_long(:,1))
xlabel('time (s)'); grid on;
ylabel('\Delta u (ft/s)');

subplot(412)
plot(t_sim_long,x_long(:,2))
xlabel('time (s)'); grid on;
ylabel('\Delta w (ft/s)');

subplot(413)
plot(t_sim_long,x_long(:,3)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\Delta q (\circ/s)');

subplot(414)
plot(t_sim_long,x_long(:,4)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\Delta \theta (\circ)');
%}
%% do same for roll convergence
% Stick-fixed Response
% roll rate disturbance

[t_sim_lat, x_lat] = ode45(@(t,x) Alat*x, [0 100], [0 pi/180 0 0]');
figure

subplot(411)
title('Lateral Stick-Fixed Response')
plot(t_sim_lat,x_lat(:,1))
xlabel('time (s)'); grid on;
ylabel('v (ft/s)');

subplot(412)
plot(t_sim_lat,x_lat(:,2))
xlabel('time (s)'); grid on;
ylabel('p (\circ/s)');

subplot(413)
plot(t_sim_lat,x_lat(:,3)*180/pi)
xlabel('time (s)'); grid on;
ylabel('r (\circ/s)');

subplot(414)
plot(t_sim_lat,x_lat(:,4)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\phi (\circ)');
% think about how to make phi settle to specific value

%% A different method for designing the gain matrix K
% Linear Quadratic Regulator (LQR)
N = zeros(4,2); % penalty of coupled state/control (almost always 0)
Q = 0.01*eye(4);    % penalty for states
R = 10*eye(2);      % penalty for control
K_lat_lqr = lqr(Alat,Blat,Q,R,N)
A_latlqr = Alat-Blat*K_lat_lqr
Lat_lqr_modes = eig(A_latlqr)

[t_latlqr, x_latlqr] = ode45(@(t,x) A_latlqr*x, [0 100], [0 pi/180 0 0]');

subplot(411)
plot(t_sim_lat,x_lat(:,1),t_latlqr,x_latlqr(:,1))
xlabel('time (s)'); grid on;
ylabel('v (ft/s)');

subplot(412)
plot(t_sim_lat,x_lat(:,2),t_latlqr,x_latlqr(:,2))
xlabel('time (s)'); grid on;
ylabel('p (\circ/s)');

subplot(413)
plot(t_sim_lat,x_lat(:,3)*180/pi,t_latlqr,x_latlqr(:,3)*180/pi)
xlabel('time (s)'); grid on;
ylabel('r (\circ/s)');

subplot(414)
plot(t_sim_lat,x_lat(:,4)*180/pi,t_latlqr,x_latlqr(:,4)*180/pi)
xlabel('time (s)'); grid on;
ylabel('\phi (\circ)');

% to set x to nonzero, replace x with x-[vector of deisred states]
% ex, phi = 5 deg
x_des = [0 0 5*pi/180 0]';