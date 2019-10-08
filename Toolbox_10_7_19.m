%% Toolbox_9_27_19
% Aerospace MQP VTOL UAV Optimizer
clc, clear variables, close all
%% Physical Characteristics
rho=1.225;          %Kg/m^3
Gamma=1.4;
T=298;              %K
V=[10,15,20,25];    %m/s
c=.3048;            %m airfoil chord
Mu=1.86e-5;         %m^2/s at 25C
R=287;              %kJ/(Kg*K)
for i=1:length(V)
    Mach(i)=V(i)/sqrt(Gamma*R*T);   %Mach #
    Re(i)=rho*V(i)*c/Mu;            %Reynold's Number
end 
%% User Inputs

use_defaults = input('Use default settings? (1 = Yes, 0 = No): ');

if use_defaults == 1
    %Air Foil Information
    air_foil_input = 'E423';
    V_input = '15';
    %Motor Information
    motor_input_v = 'AT2317_APC9x6';
    motor_input_h = 'AT2317_APC9x6';
    %Battery Information
    battery_input='TurnigyNano-Tech45C';
    %General Aircraft Information
    M_L=3;              %Kg Mass of Aircraft and Payload
    M_E=2.4;              %Kg Mass of Empty Aircraft
else
    fprintf('Choose from the the following foils NACA4412, S1223, E423, MH114, SD7026,ClarkY \n')
    air_foil_input=input('Which airfoil is being used?','s');
    V_input=input('What speed is being flown at? 10, 15, 20, or 25 m/s?','s');
    
    %Motor Information
    %VTOL
    fprintf('Choose from these motors for VTOL: MN2212KV780V20 EMAXMT3510 admiralgp54220770kv LHILHM029EMAXCF2822KV1200 DYSD3536910Kv1999kv EMAXMT2213935KV EMAXMT4008600KV EMAXMT3110700KV EMAXMT3510600KV ScorpionM3011760KV ScorpionM4010400KV AT2317_APC9x6\n')
    motor_input_v=input('What is the motors?','s');
    %Horz
    fprintf('Choose from these motors for Horz: MN2212KV780V20 EMAXMT3510 admiralgp54220770kv LHILHM029EMAXCF2822KV1200 DYSD3536910Kv1999kv EMAXMT2213935KV EMAXMT4008600KV EMAXMT3110700KV EMAXMT3510600KV ScorpionM3011760KV ScorpionM4010400KV \n')
    motor_input_h=input('What is the motors?','s');

    %Battery Information
    fprintf('Choose from these Batteries:')
    fprintf('ZIPPYCompact35C, Turnigy35C, ZIPPYCompact40C, Turnigy40C,TurnigyNano-Tech45C, TurnigyGraphene45C \n')
    battery_input=input('What is the battery?','s');
    M_L = input('Loaded mass of aircraft: ');
    M_E = input('Empty mass of aircraft: ');
end

W_L=9.81*M_L;       %N Weight Loaded
W_E=9.81*M_E;       %N Weight Empty
H_wing = c*airfoil_thickness(air_foil_input);   %M height of airfoil

air_foil_data=air_foil_find(air_foil_input,V_input);
Cl=air_foil_data(1);    %Coef. Lift
Cm=air_foil_data(2);    %Coef. Moment
V=str2double(V_input);  %m/s

%Motor Information
%VTOL
num_motors = 4;
vtol_motor_data=motor_find(motor_input_v);
%Horz
horz_motor_data=motor_find(motor_input_h);
battery_data=battery_find(battery_input);
%% Calculations
% Air Foil Calculations (assuming rectangular wing)
e=.8;                       %Oswald Efficency
Q = 0.5*V^2*rho;            %Dynamic pressure
b=W_L/(Cl*Q*c);             %m wingspan
S = b*c;                    %m^2 wing area
AR=b^2/(b*c);               % Aspect Ratio
K = (4/3)/(pi*e*AR);      %Drag polar
Cdi=Cl^2/(pi*AR*e);         % Coef. Drag Induced
A_wing = H_wing*b;
A_other = .12^2;    %M^2 Cross section of rest of aircraft (guess)
D=Q*(A_wing + A_other);     %N Drag
CD_effective = D/(Q*S);
Horz_Power=V*D;             %W Horizontal Power 
fprintf('The wingspan will be %f meters \n',b)
fprintf('The Coef. of Drag Induced is %f \n',Cdi)
fprintf('The Power needed for horizontal flight is %f W',Horz_Power')

%% Turning Calculations
Tmax = 9.81*0.001*horz_motor_data(end,2);
CL_max = air_foil_data(3);
R_min = Radius_of_turn(V,W_L,S,Tmax,Cdi,CL_max,e,AR,K);
fprintf('The minimum radius of turn for this design is %f \n', R_min)
% check if turn radius fits in the flight area
if R_min >= (100/3.28)
    fprintf('Warning: The minimum radius of turn for this design is too large for the flight area \n')
end
%% Range Estimation

% Range estimation requires first estimating the power required for takeoff
% and landing
Target_alt = 6;
x0 = [-Target_alt; 0];
Q_weights = [0.1 0.2 0.5 1 2 5 10];
% ask user if they want plots of takeoff and landing states along with
% power consumption
plot = false;
if input('Do you want plots for takeoff and landing? (1 = Yes, 0 = No)?: ') == 1
    plot = true;
end

for i = 1:numel(Q_weights)
    Q_weight = Q_weights(i);
    [E_takeoffs(i), W_max_takeoffs(i)] = takeoff_optimization(vtol_motor_data,num_motors,M_L,x0,Q_weight,plot,false);
    % Now calculate landing requirements, using lighter weight and negative x0
    [E_landings(i), W_max_landings(i)] = takeoff_optimization(vtol_motor_data,num_motors,M_E,-x0,Q_weight,plot,true);
end
% pick weight that minimizes total energy
[E_takeoff, ind_min] = min(E_takeoffs);
Q_weight_desiredt = Q_weights(ind_min);
W_max_takeoff = W_max_takeoffs(ind_min);

[E_landing, ind_min] = min(E_landings);
Q_weight_desiredl = Q_weights(ind_min);
W_max_landing = W_max_landings(ind_min);
E_TOL = E_takeoff + E_landing;
Margin = 0.8;   % fraction of battery dedicated to horizontal flight after accounting for takeoff and landing
P_autonomy = 20;    %[placeholder] power requirements for full autonomy during steady level flight
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
fprintf('Full Weight Transition \n')
[E_transition1, W_max_transition1] = transition_optimization(vtol_motor_data,horz_motor_data,num_motors,M_L,Cl,Cdi,V,S,[0.01 0.01 1],false);
fprintf('Empty Transition \n')
[E_transition2, W_max_transition2] = transition_optimization(vtol_motor_data,horz_motor_data,num_motors,M_E,Cl,Cdi,V,S,[0.01 0.01 1],false);

E_transition_net = N_transitions*E_transition1 + N_transitions_empty*E_transition2;

% capacity reserved for flight based on capacity not used for takeoff and
% transitions
E_max = Capacity*Voltage;
E_flight = Margin*E_max - E_transition_net - E_landing - E_takeoff;

% time of flight in seconds, assuming only horizontal flight
t_horz_1 = E_flight*3600/(Horz_Power + P_autonomy);
fprintf('Generous time of flight estimate: %i seconds',floor(t_horz_1))
% time of flight factoring in turning, using simple particle model of
% aircraft
if input('Do you want plots from the flight simulation? (1 = Yes, 0 = No)?: ') == 1
    plot_sim = true;
else
    plot_sim = false;
end

t_horz_2 = flight_simulator(vtol_motor_data,Cl,Cdi,S,M_L,V,E_TOL,E_transition_net,E_max,Margin,plot_sim);

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
                fprintf("Invalid Drop score \n")
            end
        else
            fprintf("Invalid Autonomy Score \n")
        end
    else
        fprintf("Invalid Originality Score \n")
    end
end

lam1 = 0.01;
lam2 = 35;
lam3 = 35;

Score = lam1*(M_L - M_E)/M_E*(t_horz_2*V) + lam2*Drop_score + lam3*Autonomy + Originality;
Score_rounded = floor(Score);
fprintf('Your predicted score for this design is %u points', Score_rounded)
