%% Toolbox_9_27_19
% Aerospace MQP VTOL UAV Optimizer
clc, clear variables, close all
%% Physical Characteristics
rho = 1.225;    %Kg/m^3 density of air at sea level
g = 9.81;       %m/s^2 acceleration of gravity    
%% User Inputs
use_defaults = input('Use default settings? (1 = Yes, 0 = No): ');

if use_defaults == 1
    %Air Foil Information
    c = .25;            %m airfoil chord
    wing_air_foil_input = 'E423';
    horz_tail_input = 'NACA0009';
    vert_tail_input = 'NACA0009';
    V = 12;
    %Motor Information
    motor_input_v = 'AT2317_APC9x6';
    motor_input_h = 'AT2317_APC9x6';
    %Battery Information
    battery_input = 'TurnigyNano-Tech45C';
    %General Aircraft Information
    M_L = 3;          % Kg Mass of Aircraft and Payload
    M_E = 2.4;        % Kg Mass of Empty Aircraft
else
    c = input('What is the chord of the airfoil in meters?: ');
    fprintf('Choose from the the following foils: NACA4412, S1223, E423, MH114, SD7026, ClarkY \n')
    wing_air_foil_input = input('Which airfoil is being used?','s');
    fprintf('Choose from the following foils: NACA0009 \n')
    horz_tail_input = input('Which airfoil is being used in the horizontal tail?','s');
    vert_tail_input = input('Which airfoil is being used in the vertical tail?','s');
    V = input('What speed is being flown at? 10, 11, 12, 15, 20, or 25 m/s?');
    
    %Motor Information
    %VTOL
    fprintf('Choose from these motors for VTOL: MN2212KV780V20 EMAXMT3510 admiralgp54220770kv LHILHM029EMAXCF2822KV1200 DYSD3536910Kv1999kv EMAXMT2213935KV EMAXMT4008600KV EMAXMT3110700KV EMAXMT3510600KV ScorpionM3011760KV ScorpionM4010400KV AT2317_APC9x6\n')
    motor_input_v = input('What is the motor?','s');
    %Horz
    fprintf('Choose from these motors for Horz: MN2212KV780V20 EMAXMT3510 admiralgp54220770kv LHILHM029EMAXCF2822KV1200 DYSD3536910Kv1999kv EMAXMT2213935KV EMAXMT4008600KV EMAXMT3110700KV EMAXMT3510600KV ScorpionM3011760KV ScorpionM4010400KV \n')
    motor_input_h = input('What is the motor?','s');

    %Battery Information
    fprintf('Choose from these Batteries:')
    fprintf('ZIPPYCompact35C, Turnigy35C, ZIPPYCompact40C, Turnigy40C,TurnigyNano-Tech45C, TurnigyGraphene45C \n')
    battery_input = input('What is the battery?','s');
    M_L = input('Loaded mass of aircraft: ');
    M_E = input('Empty mass of aircraft: ');
end

W_L = g*M_L;    %Weight Loaded, N
W_E = g*M_E;    %Weight Empty, N
H_wing = c*airfoil_thickness(wing_air_foil_input);   %M height of airfoil

air_foil_data = air_foil_find(wing_air_foil_input,V,c);
Cl = air_foil_data(1);    %Coef. Lift
Cm = air_foil_data(2);    %Coef. Moment
Cl_max = air_foil_data(3);  %Max Coef. Lift

%Motor Information
%VTOL
num_motors = 4;
motor_data = motor_find(motor_input_v);
vtol_motor_data = [0 0; motor_data(:,3) g*0.001*motor_data(:,2)];
%Horz
motor_data = motor_find(motor_input_h);
horz_motor_data = [0 0; motor_data(:,3) g*0.001*motor_data(:,2)];
battery_data = battery_find(battery_input);
%% Calculations
% Wing Calculations (assuming rectangular wing)
Om_w = 0;
% Calculate oswald efficiency based on wing properties
Q = 0.5*V^2*rho;        %Dynamic pressure
b = W_L/(Cl*Q*c);         %m wingspan
S = b*c;                %m^2 wing area
AR = b^2/(b*c);           % Aspect Ratio
e = 1.78*(1-0.045*(AR)^0.68) - 0.64;    % Oswald efficiency, formula from Aircraft Design Notes
K = (4/3)/(pi*e*AR);    %Drag polar
Cdi = Cl^2/(pi*AR*e);     % Coef. Drag Induced
% The values below assume an E423 airfoil
t2c_w = 0.125;   % ratio of airfoil thickness to chord length
x_maxt_w = 0.237;   % fraction of chord where airfoil is thickest

% Tail Calculations
% Set tail dimensions
b_ht = 0.4771;
c_ht = 0.16;
b_vt = 0.12;
c_vt = 0.16;
Om_ht = 0;
Om_vt = 0;
S_ht = b_ht*c_ht;
S_vt = b_vt*c_vt;
% The values below assume a NACA 0009 airfoil for both the vertical and
% horizontal tail
t2c_t = 0.09;   % ratio of airfoil thickness to chord length
x_maxt_t = 0.309;   % fraction of chord where airfoil is thickest

A_wing = H_wing*b;      %M^2 Cross section of wing
A_other = .12^2;        %M^2 Cross section of rest of aircraft (guess)
D=Q*(A_wing + A_other); %N Drag
CD_effective = D/(Q*S);
Horz_Power = V*D;         %W Horizontal Power 
fprintf('The wingspan will be %f meters \n',b)
fprintf('The Coef. of Drag Induced is %f \n',Cdi)
fprintf('The Power needed for horizontal flight is %f W',Horz_Power')

wing_data = [air_foil_data'; c; b; e];

% fuselage properties (these are guesses)
L_f = 0.4;  % Fuselage length, m
D_f = 0.15; % Fuselage diameter, m

% Calculate zero-lift drag
S_wets = [S; S_ht; S_vt];
Oms = [Om_w; Om_ht; Om_vt];
t2cs = [t2c_w; t2c_t; t2c_t];
x_maxts = [x_maxt_w; x_maxt_t; x_maxt_t];
char_lengths = [c; c_ht; c_vt; L_f];

Cd0 = drag_calculator(V, char_lengths, D_f, S_wets, Oms, t2cs, x_maxts)

CD = Cd0 + Cdi;
H_power = CD*Q*S*V
%% Turning Calculations
Tmax = horz_motor_data(end,2);
CL_max = air_foil_data(3);
R_min = Radius_of_turn(V,W_L,S,Tmax,Cd0,CL_max,e,AR,K);

%% Range Estimation

% Range estimation requires first estimating the power required for takeoff
% and landing
P_autonomy = 20;    %[placeholder] power requirements for full autonomy during any stage of flight

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
Margin = 0.8;   % fraction of battery dedicated to horizontal flight after accounting for takeoff and landing
if isequal(battery_input,'MaxAmps150C')
    Voltage = 14.7;
    Capacity = 3.250;
else
    Voltage = 11.1; % voltage of 3S battery
    Capacity = 2.2; % capacity of battery in Ampere-hours
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
    [E_transitions1(diagonal), W_max_transitions1(diagonal)] = transition_optimization(vtol_motor_data,horz_motor_data,num_motors,M_L,Cl,CD,x0_transition,S,P_autonomy,Q_diag,false);
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
E_flight = Margin*E_max - E_transition_net - E_landing - E_takeoff;

% time of flight in seconds, assuming only horizontal flight
t_horz_1 = E_flight*3600/(Horz_Power + P_autonomy);
fprintf('Generous time of flight estimate: %i seconds\n',floor(t_horz_1))
% time of flight factoring in turning, using simple particle model of
% aircraft
if input('Do you want plots from the flight simulation? (1 = Yes, 0 = No)?: ') == 1
    plot_sim = true;
else
    plot_sim = false;
end

t_horz_2 = flight_simulator(vtol_motor_data,wing_data,CD,M_L,V,E_TOL,E_transition_net,E_max,Margin,plot_sim);

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
fprintf('Your predicted score for this design is %u points', Score_rounded)
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
