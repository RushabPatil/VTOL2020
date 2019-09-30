function [U_r, t_flight] = endurance(W,rho,S,k,Cd0,eta_tot,E)

% calculates the predicted time of flight for a given velocity based on
% equations from "Range and Endurance Estimates for Battery-Powered
% Aircraft," Traub (2011)

% Assumptions:
% -This flight time will only be spent in steady level flight
% (realistically our score for flight time will be lower due to the need to
% turn).
% -Assume the battery capacity is based off of a discharge time of 1 hr; 
Rt = 1;
% -Assume a discharge parameter of n typical for LiPo batteries
n = 1.3;

% calculate flight velocity that results in greatest range (since scoring
% equation based on flight time*flight speed, or range)

U_r = sqrt(2*W/(rho*S)*sqrt(k/Cd0));

% time of flight comes from equation (18) in the paper mentioned above,
% only converted to seconds
t_flight = Rt^(1-n)*(eta_tot*E/((1/sqrt(rho*S))*Cd0^0.25*(2*W*sqrt(k))^1.5))^n*3600;