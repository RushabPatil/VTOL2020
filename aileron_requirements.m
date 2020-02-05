%% Aircraft Specification

m_total = 3;             % Aircraft's total mass in Kgs
AR = 6.17;               % Aspect Ratio of an aircraft
S_w = 0.3856/2;          % Total Surface Area
S_ht = 0.0763/2;         % Horizontal Tail Planform Area
S_vt =  0.0256/2;        % Vertical Tail Planform Area
lamba = 1;               % Wing's Taper Ratio
V_stall = 9.2;           % Aircraft's Stall Speed
I_xx = 0.1817;           % 
C_L_alpha_W = 4.7817;    % Stability Derivative
C_r = 0.25;              % Root Chord
b = 1.5425;              % Wing Span
rho =  1.2250;           % Atmospheric Density, kg/m^3
C_Dr = 0.9;              % Wing-horizontal tail-vertical tail rolling drag coefficient


%% Roll Maneuver Requirements
% Tentative roll requirement for our aircraft is to reach the bank angle of
% 30 degrees in 1.3 seconds

t_required = 1.3;        % required time to reach 30 degrees bank angle
phi_desired = pi/6;      % Set phi_desired equal to 30 degrees
omega_A = 20/57.3;       % A maximum aileron deflection 

%% Tentative Wing Geometry Assumptions

b_ai_to_b = 0.7;     % Inboard position is 70 percent of wing span
b_ao_to_b = 0.95;    % Outboard position is 95 percent of wing span
C_a_to_C = 0.2;      % Aileron chord is 20 percent of wing-chord
y_i = b_ai_to_b*b;   % Inboard Position of aileron
y_o = b_ao_to_b*b;   % Outboard Position of aileron
y_D =  0.4*(b/2);    % The Drag Moment Arm, typically has the value equal to
                     % 40 percent of wing's semispan

tau_a = 0.41;        % Aileron Effectiveness Parameter based on referred table

t_2 = aileron_generator(AR, S_w, S_ht, S_vt, V_stall, I_xx, C_L_alpha_W, C_r, b, rho, C_Dr, omega_A, y_i, y_o, y_D, tau_a) 



