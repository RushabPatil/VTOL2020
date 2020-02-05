function out = aileron_generator(AR, S_w, S_ht, S_vt, V_stall, I_xx, C_L_alpha_W, C_r, b, rho, C_Dr, omega_A, y_i, y_o, y_D, tau_a)

% The aileron rolling moment coefficient derivative
C_l_omega_A = (2*C_L_alpha_W*tau_a*C_r/(S_w*b))*((y_o^2 - y_i^2)/2);

% The aircraft rolling moment coefficient
C_l = C_l_omega_A*omega_A;

% approach velocity is 1.1 to 1.3 times the stall speed
V_app = 1.3*V_stall;    

% The aircraft rolling moment 
L_A = 1/2*rho*(V_app)^2*S_w*C_l*b;

% The steady-state roll rate
P_ss = sqrt((2*L_A)/(rho*(S_w + S_ht + S_vt)*C_Dr*y_D^3));

% The bank angle
phi = (I_xx/(rho*y_D^3*(S_w + S_ht + S_vt)*C_Dr))*log((P_ss)^2);

% The aircraft rate of roll rate 
P_dot = (P_ss^2)/(2*phi);

% The time taken by an aircraft to achieve the bank angle of 30 degrees
t_2 = sqrt(2*(pi/6)/P_dot);

out = t_2

end