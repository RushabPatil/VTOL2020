%% AE 4723 Steady Level Climb
clc;clear all;close all;
%% Problem 2

% Assumptions:
% Small angle approximation for angle of attack (alpha)
% high lift-to-drag ratio (L/D ~ 10; L >> D*alpha)
% can approximate lift curves as linear below stall angle
% can approximate downwash angle as varying linear with angle of attack

% inputs (properties of aircraft and airfoils)
c_bar = 5.476;  % mean chord length, ft
b = 33.8;       % wing span, ft
S = 182;        % wing area, ft^2
S_t = 42;       % tail wing area, ft^2
l_t = 16;       % distance between aerodynamic centers, ft
W = 6360;       % maximum aircraft weight, lbf
W_fuel = 2300;  % maximum weight of aircraft's fuel, lbf
T_sl = 2040;    % net sea level thrust, lbf
c_t = 1.2;      % net fuel consumption rate of engines, lb/(lbf*hr)
a = 6.06;       % wing lift curve slope, radians^-1
a_t = 4.7;      % tail lift curve slope, radians^-1
a_e = 0.4;      % elevator lift effectiveness, radians^-1
dEp_dAlpha = 0.17;  % change of downwash angle with respect to angle of attack
Ep_0 = 0;       % downwash angle at zero angle of attack, radians
Cm_ac = 0.01;   % moment coefficient about wing aerodynamic center
CL0_W = 0.1;    % wing coefficient of lift at angle of attack = 0
Cl_max = 1.4;   % maximum lift coefficient of wing
Cd0 = 0.02;     % wing drag when Cl = 0;
h_ac = 0.25;    % location of wing aerodynamic center (fraction of wing chord)
h = 0.1;        % location of CG, fraction of wing mean chord
i_t = deg2rad(-1);  % incidence angle of tail, radians
Oswald = 0.9;   % Oswald efficiency factor of wing
MoI = diag([7985 3326 11183]);    % moments of Inertia, slug ft^2

% calculated parameters
AR = b^2/S; % Aspect ratio of wing
K = 4/(3*pi*Oswald*AR); % Coefficient of Cl^2 in wing drag polar

%%
% a) Lift-to-drag
metric = 0; % in imperial units (not metric)
alts = [5000 10000];    % ft
[~,rho_sl] = Std_Atm_Model(0,0);
for i = 1:numel(alts)
    alt = alts(i);
    [~,rho]  = Std_Atm_Model(alt,metric);
    
    % find thrust at altitude
    T = T_sl*(rho/rho_sl);
    % to abide by Cl_max, find minimum airspeed velocity where aircraft can
    % perform steady level flight at the given altitude
    V_minima = sqrt(2*W/(rho*S*Cl_max));
    % take ceiling, so data points on range remain integers
    V_min(i) = ceil(V_minima);
    col = 1;
    for V = V_min(i):1000
        % find Cl_needed for steady level flight at desired velocity
        Q = 0.5*rho*V^2;    % dynamic pressure, slugs/ft^2
        CL = W/(S*Q);       % coefficient of lift
        CD = Cd0 + K*CL^2;  % coefficient of drag
        % check if thrust at altitude is sufficient to maintain steady
        % level flight at given velocity
        D = CD*S*Q;
        if D > T
            % if thrust not sufficient, go back to last whole number
            % airspeed velocity and treat that as the maximum
            V_max(i) = V - 1;
            break
        end
        L_D(i,col) = CL/CD;
        col = col + 1;
    end
end
% because data stored as different length vectors in the same matrix (w/
% zeros stored in columns to make both vectors the same length), I need to
% get rid of the zeros
V_range1 = V_min(1):V_max(1);
V_range2 = V_min(2):V_max(2);
data1 = find(L_D(1,:));
L_D1 = L_D(1,data1);
data2 = find(L_D(2,:));
L_D2 = L_D(2,data2);
figure
plot(V_range1,L_D1,V_range2,L_D2)
legend('5000 ft','10000 ft')
xlabel('Airspeed Velocity (ft/s)')
ylabel('Lift to Drag Ratio')
title('Lift-to-Drag vs. Airspeed: Cessna T37')
% b) Absolute and service ceilings

% absolute ceiling: RC_max = 0 ft/s
% service ceiling: RC_max = 100 ft/min
epsilon = 0.01; % margin of error in ceiling output, feet/sec
max_iterations = 100;

RC_des = [0 100/60]; % desired RC_max values for absolute and service ceilings respectively
alts = zeros(1,2);

for i = 1:2
    iteration = 0; % keep track of what iteration it is
    guess = 0; % guess of ceiling height, ft
    RC_target = RC_des(i); % select target for given run
    scale = 4; % power of 10 of step (i.e step size)
    dif = 0.01; % initial difference (included to avoid dividing by 0)
    condition = true;
    while  (condition)
        % increase iteration count
        iteration = iteration + 1;
        % determine max rate of climb for given altitude guess
        RC_n = Rate_of_Climb(W,S,Cd0,T_sl,K,guess,metric);
        % pull difference of previous RC from desired RC
        dif_minus = dif;
        % calculate new differece of RC from desired RC
        dif = RC_target - RC_n;
        % if difference changes sign, that means the guess of h changed
        % from an overshoot of RC to an undershoot (or vice versa). To
        % avoid endless oscillation around the desired RC, reduce the step
        % size by a factor of 10.
        if sign(dif_minus/dif) == -1
            scale = scale - 1;
        end
        step = 10^(scale);
        % check if iterations should continue. This should not be the case
        % if:
        % a) the guessed RC_n is within "epsilon" of the desired RC
        % b) the maximum number of iterations has been surpassed
        condition = -(abs(dif) >= epsilon) && -(iteration <= max_iterations);        
        % RC_max decreases with altitude. Therefore, if RC is too big,
        % increase the guess. Otherwise decrease the guess.
        if dif < 0
            guess = guess + step;
        else
            guess = guess - step;
        end    
    end
    % record the altitude
    alts(i) = guess;
end
% display answers
disp("Absolute ceiling: " + num2str(alts(1)) + " ft")
disp("Service ceiling: " + num2str(alts(2)) + " ft")

% c) Mininmum radius of turn
V_turn = 200;   % airspeed velocity during turn, ft/s
alt = 0;    % sea level, ft
g = 32.15;  % gravitational acceleration, ft/s^2
[~,rho] = Std_Atm_Model(alt,metric);
% because we are at sea level, we don't need to scale the thrust that was
% provided (given it was at sea level)
Q = 0.5*rho*V_turn^2;

% again ignore lift due to wings
W2S = W/S;
T2W = T_sl/W;
% told to assume no limit due to structural factors
% two other potential limits:
% thrust
n_maxT = sqrt(Q/(K*W2S) * (T2W - (Q*Cd0)/(W2S)))
% lift
n_maxA = Q*Cl_max/W2S
n_sl = [n_maxT n_maxA];
% find which load factor is the lower bound
n_min = min(n_sl)

% use formula for radius of turn (ft)
R = V_turn^2/(g*sqrt(n_min^2 - 1))

%% 
% d) range determination
altitude = 10000;   % cruise altitude, ft
destinations = ["Los Angeles", "Detroit", "Chicago", "Dallas", "Charlotte"]';
distances = [2553.17 574.195 822.099 1510.628 687.304]; % distances to destinations, miles

% range equation: 
% Range = (V/c_t)*(L/D)*ln(W_initial/W_final)

% assume W_final is when all fuel has been consumed
% essentially need to maximize V*(L/D)

% take data from a): "V_range2" and "L_D2"
% perform element-wise multiplication
V_L2D = V_range2.*L_D2;
% find maximum value
[optimum, opt_index] = max(V_L2D)
% note "optimum" has units of feet per second. This must be converted into
% miles per hour
opt = optimum*(3600/5280)

% split "optimum" into respective velocity and lift-to-drag ratio
V_opt = V_range2(opt_index)
L_D_opt = L_D2(opt_index)

range_max = opt/c_t * log(W/(W - W_fuel))
% find more conservative range (leaving a 10% fuel reserve)
range_c = opt/c_t * log(W/(W - 0.9*W_fuel))

% for analysis purposes, find fraction of max range that must be traveled
% from Worcester to destinations
dis_rat = distances/range_max
dis_rat_c = distances/range_c

%% 3: 
% determine angle of attack, and elevator angle to trim at velocity which
% maximizes range

% find air density at given altitude
[~,rho] = Std_Atm_Model(altitude,metric);
Q = 0.5*rho*V_opt^2;    % dynamic pressure, lbf/ft^2

% Components of CL_total
% CL_total = CL0 + CL_Alpha*alpha + CL_Deltae*Deltae

CL_0 = CL0_W + (S_t/S)*a_t*(i_t - Ep_0);
CL_Deltae = (S_t/S)*a_e;
CL_Alpha = a_t*(1 - dEp_dAlpha)*(S_t/S) + a;
CL_total = W/(Q*S);

V_H = (S_t/S)*(l_t/c_bar);      % horizontal tail volume ratio

% Components of Cm
% Cm = Cm_0 + Cm_Alpha*alpha + Cm_Deltae*Deltae
Cm_0 = Cm_ac + CL0_W*(h - h_ac) - a_t*(i_t - Ep_0)*V_H*(1 - (h - h_ac)*(c_bar/l_t));
Cm_Alpha = (a + a_t*(S_t/S)*(1 - dEp_dAlpha))*(h - h_ac) - a_t*V_H*(1 - dEp_dAlpha);
Cm_Deltae = CL_Deltae*(h - h_ac) - a_e*V_H;

% Setup linear algebra problem
A = [CL_Alpha CL_Deltae; Cm_Alpha Cm_Deltae];
B = [CL_total - CL_0; -Cm_0];

x = A\B;
Alpha = x(1);   % wing angle of attack at trim (radians)
Deltae = x(2);  % elevator angle at trim (radians)
% Convert to degrees
Alpha_d = rad2deg(Alpha)
Deltae_d = rad2deg(Deltae)
% Check that moment coefficient is zero about CG
Cm = Cm_0 + Cm_Alpha*Alpha + Cm_Deltae*Deltae
% Check that moment is zero about CG
M = 0.5*rho*V^2*S*c_bar*Cm


%% 4:   flare

% follow trajectory listed below
% x(t) = v0*t - A*t^2
% y(t) = b_m*t^m-1, m = 1:7
% z(t) = c_n*t^n-1, n = 1:7

% therefore, first and second derivatives are below
% dxdt = v0 - 2*A*t
% dydt = (m-1)*b_m*t^m-2, m = 2:7
% dzdt = (n-1)*c_n*t^n-2, n = 2:7

% d2xdt2 = -2*A
% d2ydt2 = (m-1)*(m-2)*b_m*t^m-3, m = 3:7
% d2zdt2 = (n-1)*(n-2)*c_n*t^n-3, n = 3:7

% note ranges of m, n altered so that indices of the set matrices below
% worked (as indices being 0 do not work in MATLAB)

% constants of trajectory
b_set = [30 42.8718 -24.8365 3.56385 -0.114659 1.38578e-3 0];
c_set = [-55 8.68104 4.54341 -0.976344 3.29785e-2 -4.03146e-4 0];

A = 2;
x0 = 0;     % ft
y0 = 30;    % ft
z0 = -55;   % ft
v0 = 160;   % ft/s

% aircraft mass (lbm)
m = 6360 - 2300; % assumed to be constant

% setup position and velocity vectors
x(1) = x0;
y(1) = y0;
z(1) = z0;
airspeed(1) = v0;

% initial orientation
gamma(1) = deg2rad(-3);    % initial climb angle, rad
psi(1) = deg2rad(15);      % initial heading angle, rad
phi(1) = 0;                % initial bank angle, rad
phi_i = 0;
% time range
t_range = linspace(0, 4.64466, 1000);

% preallocate for speed
z7 = zeros(1,7);
y_temp = z7;
z_temp = z7;
dydt_temp = z7;
dzdt_temp = z7;
d2ydt2_temp = z7;
d2zdt2_temp = z7;

for i = 2:numel(t_range)
    % update time step
    t = t_range(i);
    
    % update desired position and derivatives of position
    x(i) = v0*t - A*t^2;
    dxdt = v0 - 2*A*t;
    d2xdt2 = -2*A;
    % generate values of polynomial sum terms
    for n = 1:7
    y_temp(n) = b_set(n)*t^(n-1);
    z_temp(n) = c_set(n)*t^(n-1);

    dydt_temp(n) = (n-1)*b_set(n)*t^(n-2);
    dzdt_temp(n) = (n-1)*c_set(n)*t^(n-2);

    d2ydt2_temp(n) = (n-1)*(n-2)*b_set(n)*t^(n-3);
    d2zdt2_temp(n) = (n-1)*(n-2)*c_set(n)*t^(n-3);
    end
    % sum polynomials
    y(i) = sum(y_temp);
    z(i) = sum(z_temp);
    dydt = sum(dydt_temp);
    dzdt = sum(dzdt_temp);
    airspeed(i) = sqrt(dxdt^2 + dydt^2 + dzdt^2);
    d2ydt2 = sum(d2ydt2_temp);
    d2zdt2 = sum(d2zdt2_temp);
    
    % NOTE: Everything above seems to be ok, but somewhere below here I did
    % something that violated physics
    
    % calculate heading (based on trigonometry of velocity)
    psi_i = atan(dzdt/dxdt);
    psi(i) = psi_i;
    
    % calculate climb (also from trigonometry)
    gamma_i = asin(-dydt/dxdt);
    gamma(i) = gamma_i;
    
    
    % as you will see, my force balance does not end up working (I get an
    % expression stating a nonzero number equals 0, which is impossible).
    % This is likely because I should have applied the transport formula to
    % the aircraft rather than simply rotating the forces being exerted on
    % the aircraft
    
    % Create rotation matrices (for rotating from inertial to body fixed frame
    z_rot = [cos(gamma_i) sin(gamma_i) 0; -sin(gamma_i) cos(gamma_i) 0; 0 0 1];
    y_rot = [cos(psi_i) 0 -sin(psi_i); 0 1 0; sin(psi_i) 0 cos(psi_i)];
    x_rot = [1 0 0; 0 cos(phi_i) sin(phi_i); 0 -sin(phi_i) cos(phi_i)];
    % multiply rotation matrices (rotation from inertial to body-fixed)
    Rot_ib = x_rot * z_rot * y_rot;
    % force vector equals acceleration
    Forces = m*Rot_ib*[d2xdt2, d2ydt2, d2zdt2]';
    % Forces = [Thrust - Drag; W*sin(gamma)*cos(phi) - Lift; 0];
    % I need tosolve "Forces" for Thrust, alpha, and phi. However, based on
    % my original force sumation I cannot do this, as I have two equations
    % but three unknowns.
    
    % Had I more time, I would also setup a moment balance, where the net
    % moment on the center of gravity of the airplane (note: rotating about
    % the body z-axis) equals the moment of inertia in this direction*[the
    % second derivative of gamma with respect to time]. I could obtain the
    % angular acceleration of gamma, and solve for angle of attack
    % necessary to generate this corresponding moment. With angle of attack
    % obtained, I could then proceed back to my "Forces" equations and
    % obtain Thrust and phi as a function of time.
    
    
    
    
    % to prove my point, I display the first "Forces" vector
    if i == 2
        disp(Forces)
    end
end
% plot what I have
% plot angles vs. time
figure
plot(t_range, rad2deg(gamma),t_range, rad2deg(psi))
legend('gamma','psi')
xlabel('time (s)')
ylabel('angle (degrees)')
title('Climb and Heading vs. Time')
% these results seem to make some sense. Despite the sharp change in both
% angles at the first time step, both angles approach 0 by the end of the
% time range


% plot position
figure
plot3(x,y,z)
xlabel('x position (ft)');
ylabel('y position (ft)');
zlabel('z position (ft)');
title('Position Over Time');
% these results seem to make sense, as y goes to 0 by the end of the time
% range


% plot airspeed over time
figure
plot(t_range, airspeed)
xlabel('time (s)')
ylabel('airspeed (ft/s)')
title('Airspeed vs. Time')
