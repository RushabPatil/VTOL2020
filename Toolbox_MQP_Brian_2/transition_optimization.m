function [Energy_net, W_max] = transition_optimization(v_motor_name,h_motor_name,num_motors,mass,Cl,Cd,v_des,S,Q_diag,plot_trigger)
%% Optimize motor total power usage for transition between hover and horizontal flight
% v_motor_name: string corresponding to name of selected vertical motors
% h_motor_name: string corresponding to name of selected horizontal motor
% num_motors: number of VTOL motors on design
% mass: takeoff mass of aircraft
% Cl: aircraft coefficient of lift
% Cd: aircraft coefficient of drag
% v_des: desired cruise velocity, m s^-1
% Q_diag: diagonal values of Q matrix
% plot_trigger: indicates if plots should be generated
    % = true: generate plots
    % = false: do not generate plots

% necessary constants
g = 9.81;   % gravity, m s^-2
rho = 1.225;    % density of air, kg m^-3

string_v = v_motor_name + '.txt';
string_h = h_motor_name + '.txt';
% open the .txt file for the motors
fileID_v = fopen(string_v,'r');
fileID_h = fopen(string_h,'r');
% convert the contents of the .txt file to an Nx2 matrix
formatSpec = '%f %f %f %f %f';
size_points = [5 Inf];
points_v = fscanf(fileID_v,formatSpec,size_points)';
points_h = fscanf(fileID_h,formatSpec,size_points)';
% columns of useful stuff: [power, thrust(N)]
motor_data_v = [0, 0; points_v(:,3), g*0.001*points_v(:,2)];
max_thrust_v = max(motor_data_v(:,2));
motor_data_h = [0, 0; points_h(:,3), g*0.001*points_h(:,2)];
max_thrust_h = max(motor_data_h(:,2));

%% LQR situation
% original Q matrix (weighing error of all 3 states equally)
Q = diag(Q_diag);
R = eye(2);

% system properties
A = [0 1 0; 0 0 0; 0 0 0];
B = [0 0; 0 1; 1 0];
N = 0;
K = lqr(A,B,Q,R,N);
kp = K(2,1);
kd = K(2,2);
kp_vel = K(1,3);

% initial conditions
x0 = [1; 0; -v_des];

[t,x] = ode45(@(t,x) transition(t,x,kp,kd,kp_vel), [0 10], x0);

% convert each control signal into required motor thrust after accounting
% for aerodynamic forces
del_t = t(2:end) - t(1:end-1);
V_h = x(:,3);
Q_t = 0.5*rho*(V_h + v_des).*(V_h + v_des);
D_t = Cd*Q_t*S;
L_t = Cl*Q_t*S;

% calculate control signals f(t) and h(t) at each time from states
f = -kp*x(:,1) - kd*x(:,2);
h = -kp_vel*x(:,3);

Horz_thrust = mass*h + D_t;
Horz_thrust(Horz_thrust > max_thrust_h) = max_thrust_h;
Vert_thrust = mass*(f + g) - L_t;
% system can't deliver negative thrust, so replace these values with 0
Vert_thrust(Vert_thrust < 0) = 0;

% convert each thrust to a power per motor based on number of motors
% and linearly interpolate for each motor
Vert_thrust_ind = Vert_thrust/num_motors;

Error_trigger = false;
% check if maximum thrust listed by manufacturer for either motor set is
% exceeded
if max(Vert_thrust_ind) > max_thrust_v
    fprintf('Warning: too much power in each vertical motor. More motors required')
    Error_trigger = true;
end

if max(Horz_thrust) > max_thrust_h
    fprintf('Warning: too much power in horizontal motor. A different motor is required')
    Error_trigger = true;
end

if Error_trigger == true
    Energy_net = NaN;
    W_max = NaN;
    return
end

W_V_ind = interp1q(motor_data_v(:,2),motor_data_v(:,1),Vert_thrust_ind);
W_H = interp1q(motor_data_h(:,2),motor_data_h(:,1),Horz_thrust);

% numerically integrate with left-handed Riemann sum
Energy_net = del_t'*(W_H(1:end-1) + num_motors*W_V_ind(1:end-1))/3600;
W_net = W_H + W_V_ind*num_motors;

% find maximum total power consumption during transition
W_max = max(W_net);

if plot_trigger == true
    figure
    yyaxis left
    plot(t,x(:,1),'-b')
    ylabel('altitude error (m)')
    axis([0 10 -0.5 1])
    yyaxis right
    plot(t,x(:,2),'--r')
    ylabel('vertical velotiy (m/s)')
    legend('z(t)','dz/dt(t)')
    xlabel('Time (s)')
    axis([0 10 -0.25 0])
    title('Plot of Vertical States During Transition')

    figure
    plot(t,(x(:,3) + v_des),'-.g')
    ylabel('Horizontal Velocity (m/s)')
    xlabel('Time (s)')
    title('Plot of Horizontal Velocity During Transition')

    figure
    plot(t,num_motors*W_V_ind,'-b',t,W_H,'--r',t,W_net,'-.g')
    legend('P_V(t)','P_H(t)','Pnet(t)')
    title('Power Consumption During Takeoff')
    axis([0 10 0 1000])
    ylabel('Power (W)')
    xlabel('Time (s)')
end

function dxdt = transition(t,x,kp,kd,kp_vel)
    % approximation of aircraft dynamics during transition
    % x(1): vertical deviation from desired cruise altitude, m
    % x(2): vertical velocity, m s^-1
    % x(3): deviation in horizontal velocity from desired cruise velocity,
    % m s^-1
    dxdt(1,1) = x(2);
    dxdt(2,1) = -kp*x(1) - kd*x(2);
    
    h_t = -kp_vel*x(3);
    % ensure horizontal control effort does not exceed motor capacity
    Q_i = 0.5*rho*(x(3) + v_des)^2;
    D_i = Cd*Q_i*S;
    T_h = mass*h_t + D_i;
    if T_h > max_thrust_h
        h_t = (max_thrust_h - D_i)/mass;
    end
    dxdt(3,1) = h_t;
end

end