%% AE 4723 Homework 1
clc;clear all;close all;
%% Problem 1
disp("Problem 1")
% Using Gulfstream IV example from 10_24_2018 lecture, determine properties
% of ideal travel at steady, level flight (imperial units)

% inputs based on plane
S = 950;                    % ft^2
Cd0 = 0.015;                % unitless
W = 73000;                  % lbf
alts = [10000 30000 50000]; % ft
K = 0.0797;                   % unitless

% note we are NOT dealing with metric units
metric = 0;

% range of velocity inputs
V_range = 200:1:1000; % ft/s
V_length = length(V_range);

L_to_D = zeros(V_length,3);
Thrusts = zeros(V_length,3);

% determine properties for all 3 different altitudes
for i = 1:3
    alt = alts(i);
    [L_to_D(:,i),Thrusts(:,i)] = Aircraft_Performance(alt,Cd0,W,K,S,V_range,metric);
end
figure
plot(V_range,L_to_D(:,1),V_range,L_to_D(:,2),'r',V_range,L_to_D(:,3),'g');xlabel('Velocity (feet/sec)');
ylabel('Lift-to-Drag Ratio (unitsless)');
title('Steady Level Flight: Lift to Drag')
legend('10,000 ft', '30,000 ft','50,000 ft','Location','southeast')

% determine maximum lift to drag ratio on given interval of velocities, and
% find corresponging velocity

for i = 1:3
    disp("Altitude: " + num2str(alts(i)) + " ft")
    [val,ind] = max(L_to_D(:,i));
    disp("Maximum Lift-to-drag Ratio: " + num2str(val))
    v_ideal = V_range(ind);
    disp("Velocity of Maximum Lift-to-drag Ratio: " + num2str(v_ideal) + " ft/s")
end
%% Problem 2
disp(" ")
disp("Problem 2")
% plot thrust ranges against airspeed velocity for same altitudes above
figure
plot(V_range,Thrusts(:,1),V_range,Thrusts(:,2),'r',V_range,Thrusts(:,3),'g');xlabel('Velocity (feet/sec)');
ylabel('Thrust (lbf)');
title('Steady Level Flight: Thrust')
legend('10,000 ft', '30,000 ft','50,000 ft','Location','northeast')

% find ideal flight based on velocity range (may be off by a few ft/s due
% to the step size when plotting)
[L_D_max,ind_ideal] = max(L_to_D);
Thrust_min = zeros(1,3);
for col = 1:3
    ind_col = ind_ideal(col);
    Thrust_min(col) = Thrusts(ind_col,col);
end
V_ideal = V_range(ind_ideal);
disp("Cruise altitudes: " + num2str(alts) + " ft")
disp("Minimum Thrust: " + num2str(Thrust_min) + " lbf")

%% Problem 3
disp(" ")
disp("Problem 3")
% Given loss of thrust in Gulfstream IV at 30,000 ft, minimize flight path
% angle

% define aircraft properties (Start with Gulfstream IV)
W = 73000;      % lbf
S = 950;        % ft^2
Cd0 = 0.015;    % unitless
AR = 5.92;      % unitless
e = 0.9;        % unitless
K = 0.0797;       % unitless

h = 30000; % ft
v0 = 800; % ft/s
T2W = 0;
W2S = W/S;

% a) find expression and numerical value for flight path angle for steady
% descent

% find air density at given altitude
[~,rho] = Std_Atm_Model(h,0);

% find rate of descent for given airspeed velocity
R_descent = v0*(T2W - (rho*v0^2*Cd0)/(2*W2S) - (2*W2S*K)/(rho*v0^2));

% find corresponding flight path angle given airspeed velocity and rate of
% descent
gamma = asind(R_descent/v0);
disp("Rate of steady descent (V = 800 ft/s): " + num2str(R_descent) + " ft/s")
disp("Flight path angle of steady descent: " + num2str(gamma) + " degrees")
% b) If possible, find numerical value of airspeed to minimize flight path
% angle @ 30,000 ft

% brute force finding the velocity of minimum descent rate

v_values = 200:10:1000;
for i = 1:numel(v_values)
    v0 = v_values(i);
    R_descents(i) = v0*(T2W - (rho*v0^2*Cd0)/(2*W2S) - (2*W2S*K)/(rho*v0^2));
end
figure
plot(v_values,R_descents)
xlabel('Velocity (feet/sec)');ylabel('Rate of Descent (ft/s)');
title('Steady Descent, No Thrust')
% find slowest rate of descent (least negative value)
[val,ind] = max(R_descents);
v_ideal = v_values(ind);
disp("Airspeed velocity of minimum descent rate: " + num2str(v_ideal) + " ft/sec")
disp("Minimum descent rate: " + num2str(val) + " ft/sec")
%% Problem 4
disp(" ")
disp("Problem 4")
% find absolute and service ceilings of Gulfstream IV

T = 13850*2; % lbf
% other aircraft parameters reused from previous problems

% absolute ceiling: RC_max = 0 ft/s
epsilon = 0.01; % margin of error in ceiling output, feet/sec
max_iterations = 100;

RC_des = [0 100/60]; % desired RC_max values for absolute and service ceilings respectively
alts = zeros(1,2);

for i = 1:2
    disp("    Case " + num2str(i))
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
        RC_n = Rate_of_Climb(W,S,Cd0,T,K,guess,metric);
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
    disp("Number of iterations taken: " +num2str(iteration))
    disp("Power of guess step: " + num2str(scale))
end
% display answers

disp("Absolute ceiling: " + num2str(alts(1)) + " ft")
disp("Service ceiling: " + num2str(alts(2)) + " ft")

%% Problem 5
% determine expressions for accelerated climbing turn maneuver

% Start with gross motion equations for accelerated flight
% x: T - D - W*sin(gamma) = m*V_dot
% y: L*sin(phi) = m*V*cos(gamma)*psi_dot
% z: L*cos(phi) - W*cos(gamma) = m*V*gamma_dot

% Plot motion of plane (assuming properties of Gulfstream IV) to check if
% equations satisfy constant acceleration and radius of curvature

% Define aircraft properties
W = 73000;      % lbf
S = 950;        % ft^2
Cd0 = 0.015;    % unitless
K = 0.0797;     % unitless
W2S = W/S;      % lbf/ft^2

% Initial conditions
h(1) = 1000;        % altitude, ft
x(1) = 0;           % ft
y(1) = 0;           % ft
psi = 0;            % initial heading, radians
V = 300;            % initial velocity, ft/s

% Proerties of trajectory
gamma = 1;          % flight path angle, degrees
Cos = cosd(gamma);
Sin = sind(gamma);
a0 = 1;             % acceleration, ft/s^2
R = 3000;           % radius of curvature, ft
Cl = 1.6;           % coefficient of lift constant (multiplied by angle of attack to return true coefficient of lift), unitless
g = 32.15;          % acceleration of gravity, ft/s^2

% Setup numerical integration
dt = 0.01;          % seconds
time = 0:dt:240;    % seconds
range = ones(size(time));   % used to plot constants against time
% Setup vectors to record data (to plot later)
V_theoretical = V + a0*time;
V_net(1) = V;       % ft/s
V_dot(1) = a0;
psi_net(1) = psi;   % radians
Velocity(:,1) = [V*Cos*cos(psi); V*Cos*sin(psi); V*Sin];    % ft/s
r(1) = R;           % ft

for i = 2:numel(time)
    % Split velocity into directional components
    Velocity(:,i) = [V*Cos*cos(psi); V*Cos*sin(psi); V*Sin];
    % Find current position (based on previous velocity)
    x(i) = x(i-1) + Velocity(1,i-1)*dt;
    y(i) = y(i-1) + Velocity(2,i-1)*dt;
    h(i) = h(i-1) + Velocity(3,i-1)*dt;
    % Find density of atmosphere at given altitude
    [~,rho] = Std_Atm_Model(h(i),0); 
    % Dynamic pressure (lbf/ft^2)
    Q = 0.5*rho*V^2;
    % Angle of attack (radians)
    alpha = (W2S/(Q*Cl))*sqrt(Cos^2 + (V^4/(g*R)^2));
    % Bank angle (Degrees)
    phi(i) = atand(V^2/(R*g));
    % Required thrust (lbf)
    T(i) = W*(Sin + a0/g)+ Q*S*(Cd0 + K*alpha^2*Cl^2);
    % Drag experienced by aircraft at current velocity and altitude (lbf)
    D = Q*S*(Cd0 + K*(alpha*Cl)^2);
    % Lift generated by aircraft (lbf)
    L = Q*S*Cl*alpha;
    % Load factor (unitless)
    n = L/W;
    % Radius of curvature (ft); generated to see if it stays constant
    r(i) = V^2/(g*sqrt(n^2 - 1));
    % acceleration along velocity vector (ft/s^2); should equal a0
    V_dot(i) = (T(i) - D - W*Sin)/(W/g);
    % Rate of change of heading angle (rad/s)
    psi_dot = V/r(i);
    % Heading (rad)
    psi = psi + psi_dot*dt;
    % Concatentate headings
    psi_net(i) = psi;
    % Calculate airspeed (ft/s)
    V = V + V_dot(i)*dt;
    % Concatenate airspeeds
    V_net(i) = V;    
end
% Plot velocity vs. Time
figure
plot(time,V_net,'b', time, V_theoretical,'r--')
xlabel('Time (seconds)')
ylabel('Airspeed (ft/s)');
title('Velocity vs. Time')
legend('Numerically Integrated Airspeed','Theoretical Airspeed')

% Plot acceleration vs. Time
figure
plot(time,V_dot,'b', time, a0*range,'r--')
xlabel('Time (seconds)')
ylabel('Acceleration (ft/s^2)');
title('Acceleration vs. Time')
legend('Numerically Integrated Acceleration','Theoretical Acceleration')

% Plot radius of curveature vs. time
figure
plot(time,r,'b', time, R*range,'r--')
xlabel('Time (seconds)')
ylabel('Radius of Curvature (ft)');
title('R vs. Time')
legend('Numerically Integrated Radius','Theoretical Radius')

% Plot bank angle vs. time
figure
plot(time(2:end),phi(2:end))
xlabel('Time (seconds)')
ylabel('Bank Angle (degrees)');
title('Phi vs. Time')

% Plot components of velocity vs. time
figure
subplot(311);plot(time,Velocity(1,:))
ylabel('Vx (ft/s)');
title('Velocity Components vs. Time')
subplot(312);plot(time,Velocity(2,:))
ylabel('Vy (ft/s)');
subplot(313);plot(time,Velocity(3,:))
ylabel('Vz (ft/s)');
xlabel('t (s)');


% Plot position vs. time
figure
plot3(x,y,h)
xlabel('x (ft)')
ylabel('y (ft)')
zlabel('h (ft)')
title('Climbing Turn')
