%% Dynamic Stabilty Calculations
clc;clear all;close all;
% Code adapted from ADC homework
%% Properties of VTOL aircraft

% It is approximated that the CG is horizontally aligned with the main airfoil, making ltv = ltv_bars
% All airfoils on the aircraft are not swept, thus the sweep angle is the
% same regardless of where in the airfoil they are being measured

% Trim conditions
alt = 6;    % cruise altitude, m (really 6 m but approximate as sea level
theta0 = deg2rad(0); % angle of attack at trim, rad
alpha1 = theta0;
% the trig ratios of the trim angle of attack are used extensively
Ca = cos(alpha1);
Sa = sin(alpha1);
Ta = tan(alpha1);

u0 = 12;    % cruise speed, m/s (default is 12 m/s
rho = 1.225;    % air density at sea level, kg m^3
q_bar = 0.5*rho*u0^2;  % dynamic pressure, N/m^2
g = 9.81;       % gravitational acceleration, m/s^2
Mu = 1.86e-5;   % viscosity of air at 25C

% Determine Mach number, which should be essentially 0
gamma = 1.4;    % adiabatic index of a diatomic gas (air)
R = 8.3145;     % universal gas constant
M = 28.97*10^-3;      % Molar mass of air, kg/mol
T = 288.16;     % Air temperature at sea level, K
speed_of_sound = sqrt(gamma*R*T/M); % Speed of sound, m/s
Mach = u0/speed_of_sound;    % Mach number of aircraft at cruise
Beta = sqrt(1-Mach^2);  % Beta factor at cruise

% Aircraft properties - aerodynamic coefficients
Cl_alpha0 = 0.8653; % Cl at angle of attack of 0 degrees
cd0 = 0.0799;   % Zero-lift drag at trim (Cd0 = 0.0799)
cm_w = -0.19;   % Wing moment coefficient at trim

% Aircraft properties - wing properties
c_bar = 0.25;   % mean wing chord, m
b = 1.5425;     % wing span, m
lambda = 1; % wing taper ratio, c_tip/c_chord
Sweep = 0;  % sweep angle of wing, radians
Gam_w = 0;      % Dihedral angle of wing, radians
hn_wb = 0.25;   % aerodynamic center of wing airfoil, fraction of c_bar

% Aircraft properties - tail properties
b_ht = 0.4771;  % horizontal tail span, m
c_ht = 0.16;    % mean horizontal tail chord, m
Sweep_ht = 0;   % sweep angle of horizontal tail, radians
i_t = deg2rad(0); % incidence angle of horizontal tail, radians
Gam_h = 0;      % Dihedral angle of horizontal tail, radians
Gam_v = 0;      % Dihedral angle of vertical tail, radians
b_vt = 0.16;    % vertical tail span, m
c_vt = 0.16;    % mean vertical tail chord, m
Sweep_vt = 0;   % sweep angle of vertical tail, radians
etat = 0.85;    % horizontal tail dynamic pressure ratio/horizontal tail efficiency (Assume 0.85)
c_e2c_ht = 0.25;    % ratio of chord of elevator to chord of horizontal tail
tau_e = 0.45; % elevator effectiveness estimated from Napolitano's Aircraft Dynamics, 2012

% Input aircraft properties - other geometries and mass properties
m_l = 3;        % loaded aircraft mass, kg
X_cg = 0.1/0.25;  % CG location behind wing leading edge, fraction of c_bar (a.k.a. h)
lt_bar = 0.6975;   % distance between aerodynamic centers of airfoils, m
ltv_bar = 0.16;     % vertical distance between horizontal tail and CG, m
I_g_cm = [1816875 0 43157; 0 838781 0; 43157 0 2583836];    % Inertia matrix, g*cm^2
% Convert to better units (kg*m^2)
I = I_g_cm/(1000*100^2);
Ix = I(1,1);    % x-axis MoI, kg m^2
Iy = I(2,2);    % y-axis MoI, kg m^2
Iz = I(3,3);    % z-axis MoI, kg m^2
Ixz = I(1,3);   % xz-plane MoI, kg m^2
eps_w = 0;  % wing twist angle, radians
db = 0.15;  % fuselage diameter, m (rough approximation as our fuselage isn't round)

% Derived aircraft properties - geometries and mass properties
W_l = m_l*g;    % loaded aircraft weight, kg
S = b*c_bar;    % wetted wing area, m^2
Cw0 = W_l /(q_bar*S); % Non-dimensional aircraft weight
AR = b^2/S; % Aspect Ratio of wing
h_cg = X_cg*c_bar;  % CG location behind wing leading edge, m
S_ht = b_ht*c_ht;   % wetted area of horizontal tail, m^2
S_vt = b_vt*c_vt;   % wetted area of vertical tail, m^2
AR_ht = b_ht^2/S_ht;    % aspect ratio of horizontal tail
AR_vt = b_vt^2/S_vt;    % aspect ratio of vertical tail
hn_ht = lt_bar/c_bar + hn_wb;   % position of tail aerodynamic center relative to wing leading edge, normalized by mean wing aerodynamic chord
lt = lt_bar + c_bar*(hn_wb-X_cg);

% Tail volume ratios, left unsuppressed to serve as sanity checks
Vv = 2*S_vt*lt_bar/(S*b)    % Vertical tail volume ratio
% Note: this should fall between 0.02 and 0.05
Vh_bar = lt_bar*S_ht/(c_bar*S)  % Horizontal tail volume ratio
% Note: this should fall between 0.3 and 0.6

% Derived aircraft properties - aerodynamic properties
cl_aw = liftcurve(AR,Mach,Sweep);   % wing lift-curve slope
cl_a0 = liftcurve(AR,0,Sweep);      % wing lift-curve slope at mach# = 0
cl_ah = liftcurve(AR_ht,Mach,Sweep_ht); % horizontal tail lift-curve slope
% Note: vertical tail lift-curve slope is calculated later
e = 1.78*(1-0.045*(AR)^0.68) - 0.64;    % Wing Oswald efficiency, formula from AE 4770 notes
dEp_dAlpha = 4.44*sqrt(1-Mach^2)*((AR^-1 - (1 + AR^1.7)^-1)*(10 - 3*lambda)/7*((1-(ltv_bar/b))/(2*lt_bar/b)^0.33)*sqrt(cos(Sweep)))^1.19;

% calculate position of neutral point as a fraction of the wing mean chord
a_bar = cl_aw*(1 + etat*cl_ah*S_ht/(cl_aw*S)*(1 - dEp_dAlpha)); % Wing-tail effective lift curve slope
hn = hn_wb + etat*cl_ah/a_bar*Vh_bar*(1 - dEp_dAlpha);
x_np = hn*c_bar

% Leftover checks from tail sizing
x_let = lt_bar + 0.25*c_bar + 0.25*c_ht     % position of tail leading edge, defining leading edge of wing as x = 0 m
lt_des = 0.8 - 0.25*(c_bar + c_ht)          % distance between aerodynamic centers required to satisfy tail position of x = 0.8 m

%% Longitudinal Stability Derivatives

% General Constants from Aircraft Dynamics textbook (Napolitano, 2012)
C = AR*(0.5*abs(hn_wb-X_cg) + 2*(hn_wb-X_cg)^2)/(AR + 2*cos(Sweep)) + (1/24)*AR^3*tan(Sweep)^2/(AR + 6*cos(Sweep)) + 1/8;
B = sqrt(1 - Mach^2*cos(Sweep)^2);
Kq = 0.7;   % Correction coefficient (Napolitano, p.96)

% Some stability or control derivatives are assumed to be 0 either in the
% AE 4770 notes or in Napolitano's textbook
ctx1 = 0;   % Defined to be 0 (AE4770)
cmt1 = 0;   % Defined to be 0 (AE4770)
ctxu = 0;   % Defined to be 0 (AE4770)
cmtu = 0;   % Defined to be 0 (AE4770)
cdu = 0;    % Defined to be 0; As this is the change in drag wrt Mach# * Mach # this should be essentially 0 at low subsonic speeds
cmu = 0;    % Defined to be 0; should only be relevant in transonic regime anyways
cd_dele = 0;    % Defined to be 0 except for "large" horizontal tails
cd_ih = 0;      % Defined to be 0 except for "large" horizontal tails

% Other stability derivatives
cl1 = (Cw0*S + i_t*cl_aw*S_ht)/(S + S_ht);  % Coefficient of lift at trim (comes from airfoil/ Cw0)
cd1 = cd0 + cl1^2/(pi*AR*e);    % coefficient of drag at trim
cda = 2*cl1*a_bar/(pi*AR*e);
clu = (Mach^2/(1-Mach^2))*cl1;
cla = cl_aw + cl_ah*etat*S_ht/S*(1 - dEp_dAlpha);
cla_dot = 2*cl_ah*etat*S_ht/S*(hn_ht - X_cg)*dEp_dAlpha;
clq_wM0 = (0.5 + 2*abs(hn_wb - X_cg))*cl_a0;
clq_w = (AR + 2*cos(Sweep))/(2*cos(Sweep) + AR*B)*clq_wM0;
clq_h = 2*cl_ah*etat*S_ht/S*(hn_ht - X_cg);
clq = clq_w + clq_h;
cma = cl_aw*(X_cg - hn_wb) - cl_ah*etat*S_ht/S*(1-dEp_dAlpha)*(hn_ht - X_cg);
cma_dot = -cla_dot*(hn_ht - X_cg);
cm_qw_M0 = -Kq*cl_a0*cos(Sweep)*C;
cm_qw = cm_qw_M0*(1/B);
cm_qh = -2*cl_ah*etat*S_ht/S*(hn_ht - X_cg)^2;
cmq = cm_qw + cm_qh;

% Other control derivatives
cl_ih = etat*S_ht/S*cl_ah;
cl_dele = cl_ih*tau_e;
cm_ih = -cl_ah*etat*S_ht/S*(hn_ht - X_cg);
cm_dele = cm_ih*tau_e;

% determine angle of attack (should be close to 0) and elevator angle
% needed for trim (to cross-reference XFLR5 analysis)
Cl_trim = W_l/(q_bar*S);
Cm0 = cm_w - cl_ah*Vh_bar*lt/lt_bar*i_t;
AoA_trim = rad2deg((Cm0*cl_dele + cm_dele*(Cl_trim - cl1))/(a_bar*cm_dele - cl_dele*cma))
dele_trim = rad2deg(-(a_bar*Cm0 + cma*(Cl_trim - cl1))/(a_bar*cm_dele - cl_dele*cma))

%% Lateral Stability Derivatives

% General Constants from Aircraft Dynamics textbook (Napolitano, 2012)
% B is the same as in the section "Longitudinal Stability Derivatives"
D = 1 + AR*(1-B^2)/(2*B*(AR*B + 2));    % Expression aggressively simplified assuming unswept wings

% Additional geometries used by Napolitano equations (the sign of some of
% these may be wrong)
Zw_dist = 0;    % vertical distance from 25% of wing root to fuselage centerline, m
Zv_dist = b_vt/2;   % distance to point of aplication of lateral force (as vertical tail is rectangular, this is half the vertical span)
Xv_dist = lt_bar + hn_wb*c_bar - h_cg;    % lever arm of vertical tail relative to aircraft cg
d = 0.01;  % maximum fuselage height at the wing-body intersection (fudged)
ew = 0; % wing twist angle, rad (also my thoughts about this code)
r1 = 0.01;  % total height of fuselage at quarter-chord point of vertical tail, m
l_cg = 0.2; % distance from tip of fuselage to center of gravity (fudged to ignore front props, need actual value)
l_b = 0.9;  % total fuselage length, m (fudged)
z1 = 0.1;   % fuselage height @ 25% total length, m (fudged)
z2 = 0.04;  % fuselage height @ 75% total length, m (fudged)
z_max = 0.14;   % maximum fuselage height, m (fudged)
w_max = 0.12;    % maximum fuselage width, m (fudged)
Sb_s = 0.12*0.35 + 0.5*0.01;   % fuselage side surface, m^2 (extremely fudged)
bh_lb = b_ht/l_b;   % ratio of vertical tail span to fuselage length (fudged)
c_r2c_vt = 0.25;    % rudder chord as a fraction of vertical tail chord
b_r2b_vt = 0.8;     % rudder span as a fraction of vertical tail span
c_rud = c_r2c_vt*c_vt;
b_rud = b_r2b_vt*b_vt;
S_rud = c_r2c_vt*c_vt*b_rud;
Zr = b_vt/2;  % vertical distance between aircraft center of gravity and point of application of lateral force due to rudder deflection
Xr = lt + 0.75*c_vt - 0.5*c_rud;    % horizontal distance between aircraft center of gravity and point of application of lateral force due to rudder deflection

% Ratios of fuselage dimensions (necessary for Kn, obtained from figure
% 4.68 (Napolitano, 2012))
lb2Sbs = l_b^2/Sb_s;    % fudged value: ~13.6
lcg_lb = l_cg/l_b;      % fudged value: 0.25;
zmax_wmax = z_max/w_max;    % fudged value: 1.2
z_ratio = sqrt(z1/z2);  % fudged value: ~1.6

% Resulting value of Kn
Kn = 0.0005; % wing-body interference factor (fudged)

% Reynolds number of fuselage
Re_f = rho*u0*l_b/Mu;

% coefficients in rolling moment dependence on rolling rates
k_w = cl_aw*Beta/(2*pi);
k_h = cl_ah*Beta/(2*pi);

% Values required to check graphs in Aircraft Dynamics textbook (Napolitano, 2012)
Sh_Sv = S_ht/(2*S_vt);  % ratio of total wetted area of horizontal and vertical tails
c1_val = b_vt/(2*r1);
c2_val = -ltv_bar/b_vt;
Kint_value = Zw_dist/(d/2);
bv_prime2bv = 0.86; % ratio of outer span of vertical tail to inner span
% (accounting for interference with horizontal tail in a twin-fin configuration).
r1_bv = 2*r1/b_vt;
BAR_kw = Beta*AR/k_w;
BAR_kh = Beta*AR_ht/k_h;
bv_2r1 = b_vt/(2*r1);

% The coefficients obtained from the above values
K_hv = 1.06;   % relative tail size factor
c1 = 1.4;  % coefficient in effective vertical aspect ratio
c2 = 1.7;   % other coefficient in effective vertical aspect ratio (this one is fudged)
Kint = 0;   % wing-fuselage interference factor
AReff2AR = 1.3;
Cvt = 0.9; % wing+body-horizontal tail interference coefficient
cyb_veff = 4.2;   % change in cyb_v effect to account for twin tail, based on AR_vteff of 1.7
RDP_w = -0.36;  % wing rolling damping parameter
RDP_h = -0.3;   % horizontal tail rolling damping parameter
Ky_v = 1;   % factor for lateral force at vertical tail due to sideslip angle

% Some stability or control derivatives are assumed to be 0 either in the
% AE 4770 notes or in Napolitano's textbook
cntb = 0;       % Defined to be 0 (AE4770)
cy_delA = 0;    % Approximated as 0
clb_h = 0;  % Assumed to be 0 except for horizontal tails with a significant dihedral/anhedral angle
cnb_h = 0;  % Assumed to be 0 (Napolitano, 2012)
cnb_w = 0;  % Assumed to be 0 (Napolitano, 2012)

clb_cl1_sweep = 0;  % contribution due to wing sweep angle (here assumed to be 0)
clb_cl1_AR = -1.4e-3;   % contribution due to wing aspect ratio
clb_Gam = 0;    % contribution due to wing dihedral angle
Km_sweep = 1;   % compressibility correction factor due to wing sweep
Kf = 1; % fuselage correction factor (based on AR and ratio of distance to half-chord line on wingtips and wingspan)
Km_Gam = 0; % compressibility correction factor due to wing dihedral
delta_Kr = 0.88;  % Rudder span factor (assuming untapered tail and rudder spanning 80% of vertical tail
tau_r = 0.45;   % rudder effectiveness (assuming rudder chord 25% length of tail chord)
etav = 0.85;    % vertical tail efficiency (assumed to be same as horizontal tail)
Kre_l = 0.94;   % correction factor due to fuselage reynolds number (extrapolated from Figure 4.69 (Napolitano, 2012))

del_clr_Gam = 0;
clr_cl1_m0 = 0.26;  % taken from complicated graph (Napolitano Fig 4.85)
del_clr_ew = 0.014; % wing twist contribution to yaw rate effect on rolling moment

twist_correction = -3.6e-5; % correction due to wing twist angle based on distance between wing and fuselage centerline (assuming wing in line with fuselage)
dclb_Gamw = -0.0005*AR*(db/b)^2;
dclb_Zw = 1.2*sqrt(AR)*Zw_dist/(57.3*b);

AR_vteff = c1*AR_vt*(1 + K_hv*(c2 - 1));    % effective vertical tail aspect ratio

etah_expr = 0.724 + 3.06*(S_ht/S)/(1 + cos(Sweep)) + 0.4*Zw_dist/d + 0.009*AR;  % empirical expression for eta_h*(1 + dsig_db)
etav_expr = 0.724 + 3.06*(S_vt/S)/(1 + cos(Sweep)) + 0.4*Zw_dist/d + 0.009*AR;  % empirical expression for eta_v*(1 + dsig_db)

cl_av = liftcurve(AR_vteff,M,Sweep_vt);

cyb_w = -0.0001*abs(Gam_w)*57.3;
cyb_b = 0;  % assumed to be 0 because Kint is 0
cyb_v = -2*Cvt*cyb_veff*S_vt/S; % Special case because of the twin tail
cyb_h = -0.0001*abs(Gam_h)*57.3*etah_expr*S_ht/S;

clb_wb = cl1*(clb_cl1_sweep*Km_sweep*Kf + clb_cl1_AR) + Gam_w*(clb_Gam*Km_Gam + dclb_Gamw) + dclb_Zw;
clb_v = cyb_v*(Zv_dist*Ca - Xv_dist*Sa)/b;

clp_wb = RDP_w*k_w/Beta;
clp_h = 0.5*RDP_h*k_h/Beta*S_ht/S*(b_ht/b)^2;
clp_v = 2*cyb_v*(Zv_dist/b)^2;

clr_w = D*clr_cl1_m0 - del_clr_Gam*Gam_w + del_clr_ew*ew;
clr_v = -2*cyb_v*(Xv_dist*Ca + Zv_dist*Sa)*(Zv_dist*Ca - Xv_dist*Sa)/b^2;

cnp_cl1_m0 = -AR/(6*(AR + 1)); % More aggressive simplification due to unswept wings
B_alt = B*cnp_cl1_m0;
C_alt = (AR + 4)/(AR*B + 4)*B;
del_cnp_ew = -0.00038;  % effect of wing twist on Cnp, (untapered wing)

cnp_w = C_alt*cnp_cl1_m0*cl1 + del_cnp_ew*ew;
cnp_v = -2*cyb_v*(Xv_dist*Ca + Zv_dist*Sa)*(Zv_dist*Ca + Xv_dist*Sa - Zv_dist)/(b^2);

cnr_cd0 = -0.31;    % gradient of cnr based on parasitic drag (slightly fudged as CG-AC distance required isn't listed
cnr_cl1_2 = -0.3;   % gradient of cnr based on lift at trim (slightly fudged, same as above)

cnr_w = cnr_cl1_2*cl1^2 + cnr_cd0*cd0;
cnr_v = 2*cyb_v*(Xv_dist*Ca + Zv_dist*Sa)/b^2;

cnb_b = -57.3*Kn*Kre_l*Sb_s*l_b/(S*b);
cnb_v = Ky_v*abs(cl_av)*etav_expr*S_vt/S*(Xv_dist*Ca + Zv_dist*Sa)/b;

% Lateral Stability Derivatives (Napolitano equations)
clb = clb_wb + clb_h + clb_v;
clp = clp_wb + clp_h + clp_v;
clr = clr_w + clr_v;
cyb = cyb_w + cyb_b + cyb_h + cyb_v;
cyp = 2*cyb_v*(Zv_dist*Ca - Xv_dist*Sa)/b;
cyr = -2*cyb_v*(Xv_dist*Ca + Zv_dist*Sa)/b;
cnb = cnb_w + cnb_b + cnb_h + cnb_v;
cnp = cnp_w + cnp_v;
cnr = cnr_w + cnr_v;

% Lateral Control Derivatives
cl_delA = 0;    % Defined to be 0
cy_delR = abs(cl_av)*etav*S_ht/S*delta_Kr;
cl_delR = cy_delR*(Zr*cos(alpha1) - Xr*sin(alpha1))/b;
cn_delA = 0;    % Defined to be 0
cn_delR = -cy_delR*(Xr*cos(alpha1) + Zr*sin(alpha1))/b;

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

%% Dimensional Derivatives

% Longitudinal Dimensional Derivatives
% calculate products below for more concise formulae
RuS = rho*u0*S;
RuSb = RuS*b;
RuSc = RuS*c_bar;

Xu = RuS*Cw0*sind(theta0) + 0.5*RuS*Cxu;
Xw = 0.5*RuS*Cxa;
Xq = 0.25*RuSc*Cxq;
Xw_dot = 0.25*rho*c_bar*S*Cxa_dot;
Zu = -RuS*Cw0*cosd(theta0) + 0.5*RuS*Czu;
Zw = 0.5*RuS*Cza;
Zq = 0.25*RuSc*Czq;
Zw_dot = 0.25*rho*c_bar*S*Cza_dot;
Mu = 0.5*RuSc*Cmu;
Mw = 0.5*RuSc*Cma;
Mq = 0.25*RuSc*c_bar*Cmq;
Mw_dot = 0.25*rho*c_bar^2*S*Cma_dot;

% Lateral Dimensional Derivatives
Yv = 0.5*RuS*cyb;
Yp = 0.25*RuSb*cyp;
Yr = 0.25*RuSb*cyr;
Lv = 0.5*RuSb*clb;
Lp = 0.25*RuSb*b*clp;
Lr = 0.25*RuSb*b*clr;
Nv = 0.5*RuSb*cnb;
Np = 0.25*RuSb*b*cnp;
Nr = 0.25*RuSb*b*cnr;

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

% find doubling time of spiral mode
spiral_double = log(2)/min(abs(real(Lat_modes)))

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

%{

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
        
%% XFLR5 Comparison        

% Here, use stability derivatives from XFLR5 to calculate third version of
% Along matrix, comparing it to both the previously calculated matrix from
% this script and the matrix output by XFLR5

Along_X = [-0.087 0.639 0 -9.81; -1.505 -9.239 12.009 0; -0.024 -1.413 -17.314 0; 0 0 1 0];
% modify matrix with values affected by "corrected" Czu. Some of the
% relevant dimensional derivatives are not provided by XFLR5, thus their
% counterparts from the above analysis are used instead
Cmu_mod = 0.003;
Mu_mod = 0.5*RuSc*Cmu_mod;

fraction = Zu/(m_l - Zw_dot);
mod = (Mu_mod+Mw_dot*fraction)/Iy;

Along_X_mod = Along_X;
Along_X_mod(2:3,1) = [fraction; mod]
eigs_mod = eig(Along_X_mod)

% Here, use stability derivatives from XFLR5 to calculate third version of
% Alat matrix, comparing it to both the previously calculated matrix from
% this script and the matrix output by XFLR5

Alat_X =    [-.287 0.04 -12.806 9.81; -2.744 -9.239 4.017 0;...
            2.402 -2.051 -1.866 0; 0 1 0 0];

Alat1 = [Yv/m_l, Yp/m_l, (Yr/m_l)-u0, g*cosd(theta0)];
Alat2 = [(Lv/Ix_prime)+Ixz_prime*Nv, (Lp/Ix_prime)+Ixz_prime*Np, (Lr/Ix_prime)+Nr*Ixz_prime, 0];
Alat3 = [Ixz_prime*Lv+(Nv/Iz_prime), Ixz_prime*Lp+(Np/Iz_prime), Ixz_prime*Lr+(Nr/Iz_prime), 0];
Alat4 = [0 1 tand(theta0) 0];
        
% Replace cyp and clb derived stuff (Yp and Lv from original Alat matrix)        

% incorporate other coefficients needed for these replacements
cnb_mod = 0.13057; % c_nb from XFLR5
Nv_mod = 0.5*RuSb*cnb_mod;

Alat_X_mod = Alat_X;
Alat_X_mod(2:3,1) = [(Lv/Ix_prime)+Ixz_prime*Nv_mod; Ixz_prime*Lv+(Nv_mod/Iz_prime)];
Alat_X_mod(1,2) = Yp/m_l
eigs_lat_mod = eig(Alat_X_mod)

%% Rudder Sizing Check

% using stability derivatives and process outlined in Aircraft Design
% textbook (Sadraey, 2012), iterate on size of rudder

% balancing for landing in crosswind

% aircraft properties
S_fus = 0.02;   % side area of fuselage, m^2
x_fus = 0.19 - l_cg;    % location of fuselage centroid wrt cg, m
S_bat = 0.004;  % side area of battery, m^2
x_bat = 0.066 - l_cg;   % location of battery centroid wrt cg, m
% side area of VTOL motors are balanced about CG, so their location is
% irrelevant
S_mot = 0.0288*0.036;   % side area of motor (excluding props), m^2
S_mot_mount = 0.0008;   % side area of VTOL motor mounts, m^2
x_hmot = 0.365 + 0.5*0.036 - l_cg;  % location of horizontal motor centroid wrt cg, m^2
% vertical tail area already defined as S_vt and centroid location defined
% as Xv_dist
S_tube = 0.012*0.446;   % side area of carbon fiber tubes, m^2
x_tube = 0.365 + 0.5*0.446 - l_cg;  % location of tube centroid wrt cg, m

d_c_assumed = 0.25; % distance between center of side area and center of gravity, m
S_s = S_fus + S_bat + 3*S_mot + 2*S_mot_mount + S_tube + S_vt;  % aircraft side area (sum of fuselage side area and vertical tail side area), m^2
d_c = (S_fus*x_fus + S_mot*x_hmot + S_tube*x_tube + S_vt*Xv_dist)/S_s;

Cd_y = 0.8; % PLACEHOLDER aircraft side drag coefficient
cn0 = 0;    % PLACEHOLDER zero-sideslip coefficient of yaw moment
cy0 = 0;    % PLACEHOLDER zero-sideslip coefficient of yaw force

% landing properties
V_stall = 9.2;  % aircraft stall speed, m s^-1
V_W = 3;    % maximum tolerable cross wind during landing, m s^-1
V_T = sqrt(V_stall^2 + V_W^2);  % total wind-relative aircraft speed, m s^-1

Sideslip = atan(V_W/V_stall);   % maximum tolerable sideslip angle during landing, rad

F_w = 0.5*rho*V_W^2*S_s*Cd_y;   % crosswind force, N

% balance equations (12.114) and (12.115) (Sadraey, 2012)

F = @(x)    [0.5*rho*V_T^2*S*b*(cn0 + cnb*(Sideslip-x(2)) + cn_delR*x(1)) + F_w*d_c*cos(x(2));...
            0.5*rho*V_W^2*S_s*Cd_y - 0.5*rho*V_T^2*S*(cy0 + cyb*(Sideslip-x(2)) + cy_delR*x(1))];

x0 = zeros(2,1);

x_ans = fsolve(F,x0)

R_max = rad2deg(x_ans(2))

% Find servo torque required to control both rudders

L = 0.01;   % lever arm of "pull bar" about center of rotation of rudder, m

r = 0.02;   % lever arm of servo horn, m
i = 1;
del_rs = 0:0.1:20;
for del_r = deg2rad(del_rs)
    Hs(i) = 0.5*q_bar*S_rud*c_rud*sin(del_r);  % Moment about rudder, N-m
    F_b(i) = 2*Hs(i)/(L*cos(del_r));
    theta(i) = asin(L/r)*del_r;
    Ts(i) = abs(F_b(i)*r*cos(theta(i)));    
    
    % conversely, just find lateral force due to rudder deflection Y_delr
    Y(i) = Y_delr*del_r;
    i = i+1;
end

% Find max, converting to units used by servos (kg-cm)
Ts = 100*Ts/g;
theta_m = rad2deg(max(theta))
figure
plot(del_rs,Hs)
ylabel('Rudder Moments (N-m)')
yyaxis right
plot(del_rs,Ts)
ylabel('Required Torques (kg-cm)')
figure
plot(del_rs,[Y;F_b])
legend('Lateral Force','Rudder force')

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