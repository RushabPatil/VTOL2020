function RCmax = Rate_of_Climb(W,S,Cd0,T,K,alt,metric)
% Returns maximum rate of climb for a given aircraft at a given altitude
[~,rho] = Std_Atm_Model(alt,metric);

if metric == 1
    rho_sl = 1.225;
else
    rho_sl = 0.0023769;
end
T_h = T*(rho/rho_sl);

T2W = T_h/W;
W2S = W/S;
L2D_max = 1/sqrt(4*Cd0*K);

z = 1 + sqrt(1 + 3/(L2D_max*T2W)^2);
RCmax = sqrt((W2S*z)/(3*rho*Cd0))*(T2W)^(3/2)*(1 - (z/6) - 3/(2*(T2W*L2D_max)^2*z));
end

