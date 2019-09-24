% Aerospace MQP VTOL UAV Optimizer plots
% Make sure to select Reynold's Number for the Analysis
clc, clear variables
xflr5data=readtable('C:\Users\eve\Desktop\MQP_Wing\SD7026\SD7026.csv');
%% Physical Characteristics
rho=1.225; %Kg/m^3
Gamma=1.4;
R=287;              %kJ/(Kg*K)
V=[5,10,15,20,25];  %m/s
T=298;              %K
M_L=3;              %Kg Mass of Aircraft and Payload
M_e=2;              %Kg Mass of Empty Aircraft
W_L=29.43;          %N Weight Loaded
W_e=0;              %N Weight Empty
V=[10,15,20,25];    %m/s
c=.3048;            %m
Mu=1.86e-5;         %m^2/s at 25C
R=287;              %kJ/(Kg*K)
for i=1:length(V)
    Mach(i)=V(i)/sqrt(Gamma*R*T);   %Mach #
    Re(i)=rho*V(i)*c/Mu;            %Reynold's Number
end

%% Plot of CD vs. CL
figure(1)
plot(xflr5data{:,3},xflr5data{:,2})
title('CD vs. CL')
xlabel('CD')
ylabel('CL')
%% Plot of CL vs. Alpha
figure(2)
plot(xflr5data{:,1},xflr5data{:,2})
title('Alpha vs. CL')
xlabel('Alpha (Deg)')
ylabel('CD')

%% Plot of Cm vs. Alpha
figure(3)
plot(xflr5data{:,1},xflr5data{:,5})
title('Alpha vs. CMcg')
xlabel('Alpha (Deg)')
ylabel('CMcg')