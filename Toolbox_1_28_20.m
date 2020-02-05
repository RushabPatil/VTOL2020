%% Toolbox_9_27_19
% Aerospace MQP VTOL UAV Optimizer
clc, clear variables, close all

%% Load Aircraft Properties
% Load .mat file in the place of more tedious user input
load('VTOL_properties')

%% Calculations
rho = 1.225;    %Kg/m^3 density of air at sea level
g = 9.81;       %m/s^2 acceleration of gravity    

W_L = g*M_L;    % Weight Loaded, N
W_E = g*M_E;    % Weight Empty, N
H_wing = c*airfoil_thickness(wing_air_foil_input);   %M height of airfoil

air_foil_data = air_foil_find(wing_air_foil_input,V,c);
Cl = air_foil_data(1);  % Coef. Lift
Cm = air_foil_data(2);  % Coef. Moment
Cl_max = air_foil_data(3);  % Max Coef. Lift

% Motor Information
% VTOL
motor_data = motor_find(motor_input_v);
vtol_motor_data = [0 0; motor_data(:,3) g*0.001*motor_data(:,2)];
% Horizontal
motor_data = motor_find(motor_input_h);
horz_motor_data = [0 0; motor_data(:,3) g*0.001*motor_data(:,2)];
battery_data = battery_find(battery_input);

% Wing Calculations (assuming rectangular wing)

% Calculate oswald efficiency based on wing properties
Q = 0.5*V^2*rho;    % Dynamic pressure
b = W_L/(Cl*Q*c);   % wingspan, m
S = b*c;            % m^2 wing area
AR = b^2/(b*c);           % Aspect Ratio
e = 1.78*(1-0.045*(AR)^0.68) - 0.64;    % Oswald efficiency, formula from Aircraft Design Notes
K = (4/3)/(pi*e*AR);    %Drag polar
Cdi = K*Cl^2;   % Coef. Drag Induced

S_ht = b_ht*c_ht;
S_vt = 2*b_vt*c_vt;

fprintf('The wingspan will be %f meters \n',b)
fprintf('The Coef. of Drag Induced is %f \n',Cdi)

wing_data = [air_foil_data'; c; b; e];

% Calculate zero-lift drag
S_wets = [S; S_ht; S_vt];
Oms = [Om_w; Om_ht; Om_vt];
t2cs = [t2c_w; t2c_t; t2c_t];
x_maxts = [x_maxt_w; x_maxt_t; x_maxt_t];
char_lengths = [c; c_ht; c_vt; L_f];

% calculate power needed for horizontal flight
Cd0 = drag_calculator(V, char_lengths, D_f, S_wets, Oms, t2cs, x_maxts)
CD = Cd0 + Cdi;

D_horz = CD*Q*S;
P_horz = interp1(horz_motor_data(:,2),horz_motor_data(:,1),D_horz);

fprintf('The Power needed for horizontal flight is %f W',P_horz')

%% Turning Calculations
Tmax = horz_motor_data(end,2);
CL_max = air_foil_data(3);
R_min = Radius_of_turn(V,W_L,S,Tmax,Cd0,CL_max,e,AR,K);

% find time needed to roll after turn
n_max = 1.7;    % fudged from Radius_of_turn function
phi = acosd(1/n_max); % bank angle, deg

field_height = 200*0.3048;  % width of field, m

bank_dist = field_height - 2*R_min; % distance over which aircraft must be able to "un-bank" and "re-bank" from between turns
bank_time = bank_dist/V;
min_bank_rate = 2*phi/bank_time;    % min required bank rate, deg/sec
% convert to time required to bank 30 deg
max_bank_time = 30/min_bank_rate

%% Glide calculations
% Assuming no thrust, find airspeed which would minimize descent rate
T2W = 0;
W2S = W_L/S;
v_values = 8:0.1:15;
for i = 1:numel(v_values)
    v0 = v_values(i);
    R_descents(i) = v0*(T2W - (rho*v0^2*Cd0)/(2*W2S) - (2*W2S*K)/(rho*v0^2));
    if v0 == 12
        glide_slope = rad2deg(atan(R_descents(i)/v0))
    end
end
figure
plot(v_values,R_descents)
xlabel('Velocity (m/sec)');ylabel('Rate of Descent (m/sec)');
title('Steady Descent, No Thrust')
% find slowest rate of descent (least negative value)
[val,ind] = max(R_descents);
v_ideal = v_values(ind);
disp("Airspeed velocity of minimum descent rate: " + num2str(v_ideal) + " m/sec")
disp("Minimum descent rate: " + num2str(val) + " m/sec")


%% Range Estimation

% Range estimation requires first estimating the power required for takeoff
% and landing

Target_alt = 6; % crusie altitude, m
x0 = [-Target_alt; 0];
Q_scales = [0.1 0.5 1 2];%[0.1 0.2 0.5 1 2 5 10];
% ask user if they want plots of takeoff and landing states along with
% power consumption
if input('Do you want plots for takeoff and landing? (1 = Yes, 0 = No)?: ') == 1
    plot_vtol = true;
else
    plot_vtol = false;
end

for i = 1:numel(Q_scales)
    Q_scale = Q_scales(i);
    [E_takeoffs(i), W_max_takeoffs(i)] = takeoff_optimization(vtol_motor_data,num_motors,M_L,x0,P_autonomy,Q_scale,plot_vtol,false);
    % Now calculate landing requirements, using lighter weight and negative x0
    [E_landings(i), W_max_landings(i)] = takeoff_optimization(vtol_motor_data,num_motors,M_E,-x0,P_autonomy,Q_scale,plot_vtol,true);
end
% pick scale of Q that minimizes total energy for takeoff
[E_takeoff, ind_min] = min(E_takeoffs)
Q_weight_desiredt = Q_scales(ind_min);
W_max_takeoff = W_max_takeoffs(ind_min);

% pick scale of Q that minimizes total energy for landing
[E_landing, ind_min] = min(E_landings)
Q_weight_desiredl = Q_scales(ind_min);
W_max_landing = W_max_landings(ind_min);
E_TOL = E_takeoff + E_landing;
Margin = 0.75;   % fraction of battery dedicated to horizontal flight after accounting for takeoff and landing
if isequal(battery_input,'MaxAmps150C')
    % Battery used by 2018 MQP had higher voltage/capacity than allowed for
    % this year
    Voltage = 14.7;
    Capacity = 3.250;
else
    Voltage = 11.1; % voltage of 3S battery, V
    Capacity = 2.2; % capacity of battery, Amp-hours
end

% now sum total transitions and the required energy for each of them
N_transitions = 4;

% the first 2 transitions (immediately after takeoff and over the target)
% will both involve the aircraft's max mass. Any other transitions will
% only involve the empty mass of the aircraft.
N_transitions_empty = N_transitions - 2;

Q_diags = [1; 100]*ones(1,3);
%{
Q_diags =   [1 1 1; 0.1 0.1 1; 1 1 10; 10 10 10;...
            1 1 100; 0.01 0.01 1; 10 10 100; 100 100 100];
%}

% ask user if they want plots of transition states along with power
% consumption
if input('Do you want plots for transitions? (1 = Yes, 0 = No)?: ') == 1
    plot_tran = true;
else
    plot_tran = false;
end

x0_transition = [0.5 0 -V]';

for diagonal = 1:length(Q_diags(:,1))
    Q_diag = Q_diags(diagonal,:);
    [E_transitions1(diagonal), W_max_transitions1(diagonal)] = transition_optimization(vtol_motor_data,horz_motor_data,num_motors,M_L,Cl,CD,x0_transition,S,P_autonomy,Q_diag,plot_tran);
    [E_transitions2(diagonal), W_max_transitions2(diagonal)] = transition_optimization(vtol_motor_data,horz_motor_data,num_motors,M_E,Cl,CD,x0_transition,S,P_autonomy,Q_diag,plot_tran);
end
% choose the Q_diagonal for loaded and empty transitions that minimize
% total energy consumption during each transition
[E_transition1, ind_tran1] = min(E_transitions1)
Q_diag_t1 = Q_diags(ind_tran1,:);
W_max_t1 = W_max_transitions1(ind_tran1)

[E_transition2, ind_tran2] = min(E_transitions2)
Q_diag_t2 = Q_diags(ind_tran2,:);
W_max_t2 = W_max_transitions2(ind_tran2)

% capacity reserved for flight based on capacity not used for takeoff and
% transitions
E_transition_net = (N_transitions - N_transitions_empty)*E_transition1 + N_transitions_empty*E_transition2;
E_max = Capacity*Voltage;
E_flight = Margin*E_max - E_transition_net - E_TOL;

% time of flight in seconds, assuming only horizontal flight
t_horz_1 = E_flight*3600/(P_horz + P_autonomy);
fprintf('Generous time of flight estimate: %i seconds\n',floor(t_horz_1))
% time of flight factoring in turning, using simple particle model of
% aircraft
if input('Do you want plots from the flight simulation? (1 = Yes, 0 = No)?: ') == 1
    plot_sim = true;
else
    plot_sim = false;
end

t_horz_2 = flight_simulator(vtol_motor_data,wing_data,CD,Cd0,M_L,V,E_TOL,E_transition_net,E_max,Margin,P_horz,plot_sim);

%% Score Calculation
valid_inputs = false;
% only allow inputs within set range for each question
while valid_inputs == false
    Originality = input('How original is your design on a scale of 0 to 10?: ');
    if ismember(Originality,0:10)
        Autonomy = input('How many points will you get for autonomy (0-10): ');
        if ismember(Autonomy,0:10)
            Drop_score = input('What score will you get for payload drop? (-1, 0, or 1): ');
            if ismember(Drop_score,-1:1)
                valid_inputs = true;
            else
                fprintf("\nInvalid Drop score \n")
            end
        else
            fprintf("\nInvalid Autonomy Score \n")
        end
    else
        fprintf("\nInvalid Originality Score \n")
    end
end

% Weights in scoring equation
lam1 = 0.01;
lam2 = 35;
lam3 = 35;

Score = lam1*(M_L - M_E)/M_E*(t_horz_2*V) + lam2*Drop_score + lam3*Autonomy + Originality;
Score_rounded = floor(Score);
fprintf('Your predicted score for this design is %u points\n', Score_rounded)
%% Lift-to-Drag Check
Vel_range = 10:0.1:20;
[Lift_to_Drags,Thrusts] = Aircraft_Performance(0,Cd0,W_L,K,S,Vel_range,1);

figure
plot(Vel_range,Thrusts)

Power_range = interp1q(horz_motor_data(:,2),horz_motor_data(:,1),Thrusts);
yyaxis right
plot(Vel_range,Power_range,'--r')

%% Hypothetically, what does combined Takeoff/Transition require for power?

x0_combo = [-6 0 -V];
[E_combo, W_max_combo] = full_transition_optimization(vtol_motor_data,horz_motor_data,num_motors,M_L,Cl,CD,x0_combo,S,P_autonomy,[100, 100, 100],true)
