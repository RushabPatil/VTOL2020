% Aerospace MQP VTOL UAV Optimizer plots
% Make sure to select Reynold's Number for the Analysis
clc, clear variables
xflr5data10=readtable('C:\Users\eve\Desktop\MQP_Wing\SD7026\SD 7062_T1_Re0.201_M0.00_N0.0.csv');
xflr5data15=readtable('C:\Users\eve\Desktop\MQP_Wing\SD7026\SD 7062_T1_Re0.301_M0.00_N0.0.csv');
xflr5data20=readtable('C:\Users\eve\Desktop\MQP_Wing\SD7026\SD 7062_T1_Re0.401_M0.00_N0.0.csv');
xflr5data25=readtable('C:\Users\eve\Desktop\MQP_Wing\SD7026\SD 7062_T1_Re0.502_M0.00_N0.0.csv');
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
title('Coefficent of Drag to Lift')
plot(xflr5data10{:,3},xflr5data10{:,2},xflr5data15{:,3},xflr5data15{:,2},xflr5data20{:,3},xflr5data20{:,2},xflr5data25{:,3},xflr5data25{:,2})
legend('10m/s','15m/s','20m/s','25m/s')
xlabel('CD')
ylabel('CL')
%% Plot of CL vs. Alpha
figure(2)
title('Coefficent of Lift to Angle of Attack')
plot(xflr5data10{:,1},xflr5data10{:,2},xflr5data15{:,1},xflr5data15{:,2},xflr5data20{:,1},xflr5data20{:,2},xflr5data25{:,1},xflr5data25{:,2})
legend('10m/s','15m/s','20m/s','25m/s')
xlabel('Alpha (Deg)')
ylabel('Cl')

%% Plot of Cm vs. Alpha
figure(3)
title('Coefficent of Moment to Angle of Attack')
plot(xflr5data10{:,1},xflr5data10{:,5},xflr5data15{:,1},xflr5data15{:,5},xflr5data20{:,1},xflr5data20{:,5},xflr5data25{:,1},xflr5data25{:,5})
legend('10m/s','15m/s','20m/s','25m/s')
xlabel('Alpha (Deg)')
ylabel('CMcp')
%% Save
saveas(figure (1),'SD7062CDCL.jpg');  
saveas(figure (2),'SD7062CLAOA.jpg');  
saveas(figure (3),'SD7062CMAOA.jpg');