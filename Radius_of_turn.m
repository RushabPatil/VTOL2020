function R_min = Radius_of_turn(V,W,S,Tmax,Cd0,CL_max,e,AR,K)
% Determine minimum radius of turn for aircraft
rho = 1.225;    % density of air at sea level
g = 9.81;   % acceleration due to gravity
Q = 0.5*rho*V^2;
W2S = W/S;

% find factor minimizing radius of turn

n_max_t = sqrt((Q/(K*W2S))*(Tmax/W - (Q*Cd0/W2S)))
n_max_a = Q*CL_max/W2S
% assume structural limitation is irrelevant

n_max = min([n_max_t, n_max_a]);

R_min = V^2/(g*sqrt(n_max^2-1));