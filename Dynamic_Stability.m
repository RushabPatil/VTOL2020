%% Dynamic Stabilty Calculations
clc;clear all;close all;
% Code adapted from ADC homework
%% Properties of VTOL aircraft
% Trim conditions
alt = 6;    % cruise altitude, m (really 6 m but approximate as sea level
theta0 = 0; % angle of attack at trim, deg
theta0 = deg2rad(theta0);
u0 = 13;    % cruise speed, m/s (default is 12 m/s
rho = 1.225;    % air density at sea level, kg m^3
q_bar = 0.5*rho*u0^2;  % dynamic pressure, N/m^2

% Input aircraft properties
g = 9.81;       % gravitational acceleration, m/s^2
m_l = 3;        % loaded aircraft mass, kg
W_l = m_l*g;    % loaded aircraft weight, kg
c_bar = 0.25;   % mean wing chord, m
b = 1.5425;     % wing span, m (currently = 1.5425 m)
S = b*c_bar;    % wetted wing area, m^2
AR = b^2/S; % Aspect Ratio of wing
hn_wb = 0.25;   % aerodynamic center of wing airfoil, fraction of c_bar
X_cg = 0.1/0.25;  % CG location behind leading edge, fraction of c_bar (a.k.a. h)
    % Currently X_cg = 0.4935, but aim for best case this being 0.2 for
    % stability. Actually use a slighty worse value though
h_cg = X_cg*c_bar

Cl_alpha0 = 0.8653; % Cl at angle of attack of 0 degrees
lambda = 1; % taper ratio, c_tip/c_chord
Sweep = rad2deg(0);    % sweep angle, radians
% All airfoils on the aircraft are not swept, thus the sweep angle is the
% same regardless of where in the airfoil they are being measured
% Determine Mach number, which should be essentially 0
gamma = 1.4;    % adiabatic index of a diatomic gas (air)
R = 8.3145;     % universal gas constant
M = 28.97*10^-3;      % Molar mass of air, kg/mol
T = 288.16;     % Air temperature at sea level, K
speed_of_sound = sqrt(gamma*R*T/M); % Speed of sound, m/s
Mach = u0/speed_of_sound;    % Mach number of aircraft at cruise
cl_aw = liftcurve(AR,Mach,Sweep);
cl_a0 = liftcurve(AR,0,Sweep);

b_ht = 0.4771;  % horizontal tail span, m (currently 0.6 desired, 0.4771 in SolidWorks)
c_ht = 0.16;     % horizontal tail chord, m (currently 0.23 desired, 0.075 in SolidWorks)
b_vt = 0.16;   % vertical tail span, m (currently 0.3 desired?, 0.051 in SolidWorks
c_vt = 0.16;   % vertical tail chord, m (currently 0.055 desired?, 0.075 in SolidWorks)
Om_ht = 0;      % horizontal tail sweep angle, deg
Om_vt = 0;      % vertical tail sweep angle, deg
S_ht = b_ht*c_ht;   % wetted area of horizontal tail, m^2
S_vt = b_vt*c_vt;   % wetted area of vertical tail, m^2
AR_ht = b_ht^2/S_ht;
AR_vt = b_vt^2/S_vt;
i_t = deg2rad(0); % incidence angle of tail, radians
% both components of tail have no sweep
Sweep_ht = 0;
Sweep_vt = 0;
% find tail lift-curve slope
cl_ah = liftcurve(AR_ht,Mach,Sweep_ht)

ltv_bar = 0.16;     % vertical distance between horizontal tail and CG, m
% It is approximated that the CG is horizontally aligned with the main airfoil, making ltv = ltv_bars
etat = 0.85;    % horizontal tail dynamic pressure ratio/horizontal tail efficiency (Assume 0.85)
e = 1.78*(1-0.045*(AR)^0.68) - 0.64;    % Oswald efficiency, formula from Aircraft Design Notes
lt_bar = 0.6975;   % distance between aerodynamic centers of airfoils, m
hn_ht = lt_bar/c_bar + hn_wb;   % position of tail aerodynamic center relative to wing leading edge, normalized by mean wing aerodynamic chord
lt = lt_bar + c_bar*(hn_wb-X_cg);

x_let = lt_bar + 0.25*c_bar + 0.25*c_ht    % position of tail leading edge, defining leading edge of wing as x = 0 m0
lt_des = 0.8 - 0.25*(c_bar + c_ht)
% Inertia matrix, g*cm^2
I_g_cm = [1816875 0 43157; 0 838781 0; 43157 0 2583836];
% Convert to better units (kg*m^2)
I = I_g_cm/(1000*100^2);
Ix = I(1,1);    % x-axis MoI, kg m^2
Iy = I(2,2);    % y-axis MoI, kg m^2
Iz = I(3,3);    % z-axis MoI, kg m^2
Ixz = I(1,3);   % xz-plane MoI, slug ft^2

% calculate parameters needed for stability derivatives
% Determine tail volume ratios as a sanity check
Vv = 2*S_vt*lt_bar/(S*b)    % Vertical tail volume ratio
% Note: this should fall between 0.02 and 0.05
Vh_bar = lt_bar*S_ht/(c_bar*S)  % Horizontal tail volume ratio
% Note: this should fall between 0.3 and 0.6

dEp_dAlpha = 4.44*sqrt(1-Mach^2)*((AR^-1 - (1 + AR^1.7)^-1)*(10 - 3*lambda)/7*((1-(ltv_bar/b))/(2*lt_bar/b)^0.33)*sqrt(cos(Sweep)))^1.19;
a_bar = cl_aw*(1 + etat*cl_ah*S_ht/(cl_aw*S)*(1 - dEp_dAlpha)); % Wing-tail effective lift curve slope
% calculate position of neutral point as a fraction of the wing mean chord
hn = hn_wb + etat*cl_ah/a_bar*Vh_bar*(1 - dEp_dAlpha)
x_np = hn*c_bar

c_e2c_ht = 0.25;    % ratio of chord of elevator to chord of horizontal tail
tau_e = 0.45; % estimated from Napolitano's Aircraft Dynamics, 2012
% Other constants
Cw0 = W_l /(q_bar*S); % Non-dimensional weight

%% Stability Derivatives
% Steady State
% General Constants from (Napolitano, 2012)
C = AR*(0.5*abs(hn_wb-X_cg) + 2*(hn_wb-X_cg)^2)/(AR + 2*cos(Sweep)) + (1/24)*AR^3*tan(Sweep)^2/(AR + 6*cos(Sweep)) + 1/8;
B = sqrt(1 - Mach^2*cos(Sweep)^2);

cl1 = (Cw0*S + i_t*cl_aw*S_ht)/(S + S_ht);  % Coefficient of lift at trim (comes from airfoil/ Cw0)

cd0 = 0.0799;   % Zero-lift drag at trim (Cd0 = 0.0588)
cd1 = cd0 + cl1^2/(pi*AR*e);    % coefficient of drag at trim

ctx1 = 0;       % Defined to be 0
cmt1 = 0;       % Defined to be 0
% Other stability derivatives, using equations from AE 4770 notes
cdu = 0;        % Defined to be 0; As this is the change in drag wrt Mach# * Mach # this should be essentially 0
cda = 2*cl1*a_bar/(pi*AR*e);
ctxu = 0;
clu = (Mach^2/(1-Mach^2))*cl1;
cla = cl_aw + cl_ah*etat*S_ht/S*(1 - dEp_dAlpha);
cla_dot = 2*cl_ah*etat*S_ht/S*(hn_ht - X_cg)*dEp_dAlpha;

clq_wM0 = (0.5 + 2*abs(hn_wb - X_cg))*cl_a0;
clq_w = (AR + 2*cos(Sweep))/(2*cos(Sweep) + AR*B)*clq_wM0;
clq_h = 2*cl_ah*etat*S_ht/S*(hn_ht - X_cg);
clq = clq_w + clq_h;
cmu = 0;    % Defined to be 0; should only be relevant in transonic regime anyways
% NOTE: the above equation may be wrong, as 'h' (with an unknown meaning)
% is in the AE 4770 notes instead of 'X_cg'
cma = cl_aw*(X_cg - hn_wb) - cl_ah*etat*S_ht/S*(1-dEp_dAlpha)*(hn_ht - X_cg);
cma_dot = -cla_dot*(hn_ht - X_cg);

Kq = 0.7;   % Correction coefficient (Napolitano, p.96)
cm_qw_M0 = -Kq*cl_a0*cos(Sweep)*C;
cm_qw = cm_qw_M0*(1/B);

cm_qh = -2*cl_ah*etat*S_ht/S*(hn_ht - X_cg)^2;
cmq = cm_qw + cm_qh;
cmtu = 0;   % Defined to be 0

% Control Derivatives (From Napolitano, 2012)
cd_dele = 0;    % Defined to be 0 except for "large" horizontal tails
cd_ih = 0;      % Defined to be 0 except for "large" horizontal tails
cl_ih = etat*S_ht/S*cl_ah;
cl_dele = cl_ih*tau_e;
cm_ih = -cl_ah*etat*S_ht/S*(hn_ht - X_cg);
cm_dele = cm_ih*tau_e;

% determine elevator angle needed for trim
Cl_trim = W_l/(q_bar*S);
cm_w = -0.19;
Cm0 = cm_w - cl_ah*Vh_bar*lt/lt_bar*i_t;

AoA_trim = rad2deg((Cm0*cl_dele + cm_dele*(Cl_trim - cl1))/(a_bar*cm_dele - cl_dele*cma))
dele_trim = rad2deg(-(a_bar*Cm0 + cma*(Cl_trim - cl1))/(a_bar*cm_dele - cl_dele*cma))

% Table B6.7
% Lateral Stability Derivatives
clb = 0;        % Defined to be 0
clp = 0;        % Defined to be 0
clr = 0;        % Defined to be 0
cyb = 0;        % Defined to be 0
cyp = 0;        % Defined to be 0
cyr = 0;        % Defined to be 0
cnb = 0;        % Defined to be 0
cntb = 0;       % Defined to be 0
cnp = 0;        % Defined to be 0
cnr = 0;        % Defined to be 0
% Lateral Control Derivatives
cl_delA = 0;    % Defined to be 0
cl_delR = 0;    % Defined to be 0
cy_delA = 0;    % Defined to be 0
cy_delR = 0;    % Defined to be 0
cn_delA = 0;    % Defined to be 0
cn_delR = 0;    % Defined to be 0
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
Cmu = cmu;
Cma = cma;
Cma_dot = cma_dot;
Cmq = cmq;
Cm_dele = cm_dele;
Cm_delp = cmtu + 2* cmt1;
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

%% Longitudinal Linear Model
Along1 = [Xu/m_l Xw/m_l 0 -g*cosd(theta0)];
Along2 = [Zu Zw Zq+m_l*u0 -W_l*sind(theta0)]/(m_l-Zw_dot);
Along3 =    [Mu+(Mw_dot*Zu)/(m_l-Zw_dot), Mw+(Mw_dot*Zw)/(m_l-Zw_dot),...
            Mq+(Mw_dot*(Zq+m_l*u0))/(m_l-Zw_dot), -(Mw_dot*W_l*sind(theta0))/(m_l-Zw_dot)]/Iy;
Along4 = [0 0 1 0];
Along = [Along1; Along2; Along3; Along4]

% Eigenvalues of Along
Long_modes = eig(Along)
% 1) complex pair with small magnitude real part: Phugoid
% 2) complex pair with larger magnitude real part: Short period

% Assess settling time and period of these oscillations
[Re_Short, Ind_Short] = min(real(Long_modes));
[Re_Phugoid, Ind_Phugoid] = max(real(Long_modes));

% Short period mode:
t_s_Short = -4/Re_Short
Period_Short = abs(2*pi/imag(Long_modes(Ind_Short)))
% Phugoid mode:
if real(Long_modes(3)) < 0
    t_s_Phugoid = -4/Re_Phugoid
    Period_Phugoid = abs(2*pi/imag(Long_modes(Ind_Phugoid)))
else
    fprintf('WARNING: Aircraft is dynamically unstable (Phugoid mode does not decay)')
end

% Longitudinal Dimensional Control Derivatives
QS = 0.5*rho*u0^2*S;
X_dele = Cx_dele*QS;
X_delp = Cx_delp*QS;
Z_dele = Cz_dele*QS;
Z_delp = Cz_delp*QS;
M_dele = Cm_dele*QS*c_bar;
M_delp = Cm_delp*QS*c_bar;

Blong = [X_dele/m_l, X_delp/m_l; Z_dele/(m_l-Zw_dot), Z_delp/(m_l-Zw_dot);
        M_dele/Iy+Mw_dot*Z_dele/(Iy*(m_l-Zw_dot)), M_delp/Iy+Mw_dot*Z_delp/(Iy*(m_l-Zw_dot));
        0, 0];





%{
%% Lateral Linear Model
Ix_prime = (Ix*Iz-Ixz^2)/Iz;
Iz_prime = (Ix*Iz-Ixz^2)/Ix;
Ixz_prime = Ixz/(Ix*Iz-Ixz^2);

Alat1 = [Yv/m_l, Yp/m_l, (Yr/m_l)-u0, g*cosd(theta0)];
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

Blong = [X_dele/m_l, X_delp/m_l; Z_dele/(m_l-Zw_dot), Z_delp/(m_l-Zw_dot);
        M_dele/Iy+Mw_dot*Z_dele/(Iy*(m_l-Zw_dot)), M_delp/Iy+Mw_dot*Z_delp/(Iy*(m_l-Zw_dot));
        0, 0];
Blat =  [Y_dela/m_l, Y_delr/m_l; L_dela/Ix_prime+Ixz_prime*N_dela, L_delr/Ix_prime+Ixz_prime*N_delr;
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

% isolate elevator to pitch angle
long_sys_el2pitch = ss(Along, Blong(:,1)*pi/180,[0 0 0 1], 0);
% C = [0 0 0 1] picks out 4th state variable (pitch angle) for observation
% B multiplied to pretend elevator is in degrees
figure('Name','Elevator-Pitch Angle');
subplot(211);impulse(long_sys_el2pitch);title('Impulse Response')
subplot(212);step(long_sys_el2pitch);title('Step Response')
% isolate thrust to speed
long_sys_t2speed = ss(Along, Blong(:,2), [1 0 0 0], 0);
figure('Name','Thrust-Speed');
subplot(211);impulse(long_sys_t2speed);title('Impulse Response')
subplot(212);step(long_sys_t2speed);title('Step Response')
% isoltate aileron to roll response
lat_sys_a2roll = ss(Alat, Blat(:,1)*pi/180, [0 0 0 1],0);
figure('Name','Aileron-Roll Angle');
subplot(211);impulse(lat_sys_a2roll);title('Impulse Response')
subplot(212);step(lat_sys_a2roll);title('Step Response')
lat_sys_a2rrate = ss(Alat, Blat(:,1)*pi/180, [0 1 0 0],0);
figure('Name','Aileron-Roll Rate');
subplot(211);impulse(lat_sys_a2rrate);title('Impulse Response')
subplot(212);step(lat_sys_a2rrate);title('Step Response')
% isolate rudder to yaw angle
% add psi_dot to Alat, Blat
Alat_with_yaw = [Alat zeros(4,1); 0 0 sec(theta0) 0 0];
Blat_with_yaw = [Blat; 0 0];
lat_sys_r2yaw = ss(Alat_with_yaw, Blat_with_yaw(:,2)*pi/180, [0 0 0 0 1],0);
figure('Name','Rudder-Yaw');
subplot(211);impulse(lat_sys_r2yaw);title('Impulse Response')
subplot(212);step(lat_sys_r2yaw);title('Step Response')
%% altitude hold control

% start with previous controller and hope it works
% treat altitude as linear model
Along_with_altitude = [Along zeros(4,1); -sin(theta0) cos(theta0) 0 -u0*cos(theta0) 0];    % 5x5
Blong_with_altitude = [Blong; zeros(1,2)];

K_altitude_hold = lqr(Along_with_altitude,Blong_with_altitude,0.01*eye(5), 10*eye(2), zeros(5,2))

% check how gain works
Along_alt_hold = Along_with_altitude - Blong_with_altitude*K_altitude_hold;

% airspeed disturbance w/o control
[t_sim_long_noC, x_long_noC] = ode45(@(t,x) Along_with_altitude*x, [0 400], [10 0 0 0 0]');

% airspeed disturbance with control
[t_sim_long, x_long] = ode45(@(t,x) Along_alt_hold*x, [0 400], [10 0 0 0 0]');
% plot control action
control_long = (-K_altitude_hold*x_long')';
figure('Name','Longitudinal Response')
subplot(511)
plot(t_sim_long_noC,x_long_noC(:,1),t_sim_long,x_long(:,1))
title('Longitudinal Altitude Response: With and Without Control')
xlabel('time (s)'); grid on;ylabel('\Delta u (ft/s)');
subplot(512)
plot(t_sim_long_noC,x_long_noC(:,2),t_sim_long,x_long(:,2))
xlabel('time (s)'); grid on;ylabel('\Delta w (ft/s)');
subplot(513)
plot(t_sim_long_noC,x_long_noC(:,3)*180/pi,t_sim_long,x_long(:,3)*180/pi)
xlabel('time (s)'); grid on;ylabel('\Delta q (\circ/s)');
subplot(514)
plot(t_sim_long_noC,x_long_noC(:,4)*180/pi,t_sim_long,x_long(:,4)*180/pi)
xlabel('time (s)'); grid on;ylabel('\Delta \theta (\circ)');
subplot(515)
plot(t_sim_long_noC,x_long_noC(:,5),t_sim_long,x_long(:,5))
xlabel('time (s)'); grid on;ylabel('\Delta z (ft)');

figure('Name','Longitudinal Altitude Hold Control')
subplot(211)
plot(t_sim_long(1:350),control_long(1:350,1)*180/pi)
title('Elevator Control')
xlabel('time (s)'); grid on;
ylabel('\Delta e (\circ)');

subplot(212)
plot(t_sim_long,control_long(:,2))
title('Thrust Control')
xlabel('time (s)'); grid on;
ylabel('\Delta t (lb)');
%% A different method for designing the gain matrix K
% Linear Quadratic Regulator (LQR)
N = zeros(5,2); % penalty of coupled state/control (almost always 0)
Q = 0.01*eye(5);    % penalty for states
R = 10*eye(2);      % penalty for control
K_lat_lqr = lqr(Alat_with_yaw,Blat_with_yaw,Q,R,N)
A_latlqr = Alat_with_yaw-Blat_with_yaw*K_lat_lqr
Lat_lqr_modes = eig(A_latlqr)

% to set x to nonzero, replace x with x-[vector of deisred states]
% ex, phi = 5 deg
x_des = [0 0 0 5*pi/180 0]';
% open loop response
[t_lat, x_lat] = ode45(@(t,x) Alat_with_yaw*x, [0 100], [0 pi/180 0 0 0]');
% closed loop response with desired nonzero roll angle
[t_latlqr, x_latlqr] = ode45(@(t,x) A_latlqr*(x-x_des), [0 100], [0 pi/180 0 0 0]');
figure('Name','Roll hold')
subplot(511)
plot(t_lat,x_lat(:,1),t_latlqr,x_latlqr(:,1))
title('Lateral Response: Open Loop and Roll Hold Controller')
xlabel('time (s)'); grid on;ylabel('v (ft/s)');
subplot(512)
plot(t_lat,x_lat(:,2),t_latlqr,x_latlqr(:,2)*180/pi)
xlabel('time (s)'); grid on;ylabel('p (\circ/s)');
subplot(513)
plot(t_lat,x_lat(:,3),t_latlqr,x_latlqr(:,3)*180/pi)
xlabel('time (s)'); grid on;ylabel('r (\circ/s)');
subplot(514)
plot(t_lat,x_lat(:,4),t_latlqr,x_latlqr(:,4)*180/pi)
xlabel('time (s)'); grid on;ylabel('\phi (\circ)');
subplot(515)
plot(t_lat,x_lat(:,5),t_latlqr,x_latlqr(:,5)*180/pi)
xlabel('time (s)'); grid on;ylabel('\psi (\circ)');
% determine if sideslip is suppressed to 0
% beta = asin(v/V)
beta_lqr = asind(x_latlqr(:,1)/u0);
figure('Name','Sideslip')
plot(t_latlqr,beta_lqr)
title('Sideslip Angle Due to Roll Controller')
xlabel('time (s)'); grid on;
ylabel('\beta (\circ)')

% plot control action
control_lat = (-K_lat_lqr*x_latlqr')';
figure('Name','Lateral Altitude Hold Control')
subplot(211)
plot(t_latlqr,control_lat(:,1)*180/pi)
title('Aileron Control')
xlabel('time (s)'); grid on;
ylabel('\Delta a (\circ)');

subplot(212)
plot(t_latlqr,control_lat(:,2)*180/pi)
title('Rudder Control')
xlabel('time (s)'); grid on;
ylabel('\Delta r (\circ)');
%}
        
%% Functions
function cl_alpha = liftcurve(AR,M,Sweep)
% Determines lift-curve slope of wing/tail
% INPUTS:
% AR: aspect ratio of lifting surface
% M: Mach number of airspeed
% Sweep: sweep angle of quarter-chordline of lifting surface, radians
    if AR < 4
        k = 1 + AR*(1.87-2.33e-4*Sweep)/100;
    else
        k = 1 + (8.2 - 2.3*Sweep - AR*(0.22 - 0.153*Sweep))/100;
    end
    % Technically "Sweep" in the equations above is on the leading edge.
    % For our purposes this is fine, as no wings have any sweep to speak of
    cl_alpha = (2*pi*AR)/(2 + sqrt((AR/k)^2*(1 - M^2)*(1 + tan(Sweep)^2/(1 - M^2)) + 4));
end
        