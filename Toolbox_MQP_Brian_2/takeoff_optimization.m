function [Energy_net, W_max] = takeoff_optimization(motor_input,num_motors,mass,x0,scale,plot_trigger,landing)
%% Optimize motor total power usage for takeoff and landing
% motor_name: string corresponding to name of selected motor
% num_motors: number of VTOL motors on design
% mass: mass of aircraft, kg
% x0: initial conditions, taking the form [distance above target altitude;
% initial vertical velocity], [m; m s^-1]
    % NOTE: a negative x0(1) value is for taking off while a positive x0(1)
    % value is for landing
% scale: relative weight of Q (steady-state error) vs. R (control effort)
% in LQR (norm of entries in Q/norm of entries in R)
% plot_trigger: indicates if plots should be generated
    % = true: generate plots
    % = false: do not generate plots
% landing: indicates if the landing simulatino should be run instead
    % = true: aircraft is landing
    % = false: aircraft is taking off

    
% before anything is done, check to make sure x0 and landing match their
% desired situations
if (x0(1) < 0) == landing
    fprintf("Invalid initial conditions")
    Energy_net = NaN;
    W_max = NaN;
    return
end

% as of writing a .txt file only exists for the EMAXMT3510_600KV
string = motor_input + '.txt';

% open the corresponding .txt file
fileID = fopen(string,'r');

% convert the contents of the .txt file to an Nx2 matrix
formatSpec = '%f %f %f %f %f';
size_points = [5 Inf];
points = fscanf(fileID,formatSpec,size_points)';

g = 9.81;   % gravity, m s^-2
% columns of useful stuff: [power, thrust(N)], attaching [0, 0] to the top
motor_data = [0,0; points(:,3), g*0.001*points(:,2)];

% find maximum power listed by manufacturer
thrust_limit = max(motor_data(:,2));
% we will linearly interpolate within the motor data later

%% LQR situation

% setup state space model of quadcopter during takeoff
% x1: vertical position, m
% x2: vertical velocity, m s^-1

A = [0 1; 0 0];
B = [0; 1];
Q = scale*eye(2);
R = 1;
N = 0;

% use LQR to determine coefficients to drive control thrust
K = lqr(A,B,Q,R,N);
kp = K(1);
kd = K(2);

[t,x] = ode45(@(t,x) takeoff(t,x,kp,kd), [0 20], x0);

% calculate control signal u(t) at each time from states
u = -kp*x(:,1) - kd*x(:,2);
% convert control signal into total thrust from the quadcopter's vertical
% motors, dividing by number of motors
T_ind = mass*(u + g)/num_motors;
% if landing, zero out the thrust at any instant where the aircraft
% altitude = 0 meters
if landing == true
    T_ind(x(:,1) <= 0) = 0;
    % in addition, make plotting sensical by setting nonzero altitudes to 0
    xplot1 = x(:,1);
    xplot1(xplot1 < 0) = 0;
    x(:,1) = xplot1;
end

% linearly interpolate T_ind to yield W_ind (Note: if power required by a
% motor exceeds range provided by manufacturer, a cell will be set to NaN)
W_ind = interp1q(motor_data(:,2),motor_data(:,1),T_ind);
W_net = W_ind*num_motors;
W_max = max(W_net);

v = x(:,2);
del_t = t(2:end) - t(1:end-1);
Acceleration = (v(2:end) - v(1:end-1))./del_t;

if max(T_ind) > thrust_limit
    Energy_net = 'Motors are being overworked. Change number of motors or kind of motors';
else
    Energy_net = sum(W_net(1:end-1).*del_t)/3600;
end

if plot_trigger == true
    figure
    subplot(2,1,1)
    plot(t,x(:,1),'-b',t,x(:,2),'--r',t(1:end-1),Acceleration,'-.g')
    legend('x(t)','dx/dt(t)','d^2x/dt^2(t)','location','best')
    title('Plot of States During Takeoff')

    subplot(2,1,2)
    plot(t,W_net)
    title('Power Consumption During Takeoff (Linear Interpolation)')
    ylabel('Power (W)')
    xlabel('Time (s)')

    figure
    plot(W_ind, T_ind,'.b',motor_data(:,1),motor_data(:,2),"*r")
    legend('Predicted Relationships','Manufacturer Data','location','best')
    xlabel('Power (W)')
    ylabel('Thrust (N)')
    title('Thrust vs. Power of EMAX MT3510 600KV')
end

function dxdt = takeoff(t,x,kp,kd)
    dxdt(1,1) = x(2);
    dxdt(2,1) = -kp*x(1) - kd*x(2);
end
end