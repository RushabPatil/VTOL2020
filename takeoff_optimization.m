function [Energy_net, W_max] = takeoff_optimization(motor_info,num_motors,mass,x0,scale,plot_trigger,landing)
%% Optimize motor total power usage for takeoff and landing
% motor_info: Nx5 array containing motor performance while varying input
% current
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
if (x0(1) < 0) && landing
    fprintf("Invalid initial conditions")
    Energy_net = NaN;
    W_max = NaN;
    return
end

g = 9.81;   % gravity, m s^-2
% columns of motor_data: [power, thrust(N)], attaching [0, 0] to the top
motor_data = [0,0; motor_info(:,3), g*0.001*motor_info(:,2)];

% find maximum thrust listed by manufacturer
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

[t,x] = ode45(@(t,x) takeoff(t,x,kp,kd,thrust_limit), [0 20], x0);

% calculate control signal u(t) at each time from states
u = -kp*x(:,1) - kd*x(:,2);
% convert control signal into total thrust from the quadcopter's vertical
% motors, dividing by number of motors
T_ind = mass*(u + g)/num_motors;
% note: the simulation capped T_ind at the maximum thrust per motor
if max(T_ind) >= thrust_limit
    fprintf('Warning: Current conditions involve motors running at max throttle')
    T_ind(T_ind >= thrust_limit) = thrust_limit;
end
% zero out any negative thrust requests
T_ind(T_ind < 0) = 0;
% if landing, zero out the thrust at any instant where the aircraft
% altitude = 0 meters
if landing
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
% find settling time of system
margin = 0.02;
% find last point where x(1) exceeds margin*x0(1) difference from
% steady-state. t_s is 1 index after this
settle_ind = find(abs(x(:,1)) >= margin*abs(x0(1)), 1, 'last');
t_s = t(settle_ind);
x_s = x(settle_ind);
% if landing just numerically integrate everything, as thrust will hit 0 as
% soon as the aircraft touches the ground
if landing
    Energy_net = max(cumtrapz(t,W_net))/3600;
else
    % if taking off, only numerically integrate from t = 0 to t_s, where t_s is
    % the settling time to reach oscillations less than margin % of abs(x0(1))
    Energy_net = max(cumtrapz(t(1:settle_ind),W_net(1:settle_ind)))/3600;
end
Acceleration = diff(x(:,2))./diff(t);

if plot_trigger == true
    if landing
        state = "Landing";
    else
        state = "Takeoff";
    end
    fprintf("The system settles after %f seconds",t_s);
    
    % if told to plot, generate plots of states and power during takeoff
    s_bound = [-6 8];
    t_bound = [0 12];
    figure
    subplot(2,1,1)
    plot(t,x(:,1),'-b',t,x(:,2),'--r',t(1:end-1),Acceleration,'-.g',[t_s t_s],s_bound,'--k')
    legend('x(t)','dx/dt(t)','d^2x/dt^2(t)')
    title("Plot of States During " + state + ": Scale = " + num2str(scale))
    yticks(s_bound(1):2:s_bound(2))
    axis([t_bound, s_bound])
    
    subplot(2,1,2)
    plot(t,W_net,[t_s t_s],[200 800],'--k')
    title("Net Power Consumption During " + state + " (Linear Interpolation)")
    ylabel('Power (W)')
    xlabel('Time (s)')
    yticks(200:100:800)
    axis([t_bound, 200 800])
%{
    figure
    plot(W_ind, T_ind,'.b',motor_data(:,1),motor_data(:,2),'*r')
    legend('Predicted Relationships','Manufacturer Data','location','best')
    xlabel('Power (W)')
    ylabel('Thrust (N)')
    title('Thrust vs. Power of Motor')
%}    
end

function dxdt = takeoff(t,x,kp,kd,thrust_limit)
    if landing && x(1) <= 0
        dxdt = zeros(2,1);
    else
    dxdt(1,1) = x(2);
    u_i = -kp*x(1) - kd*x(2);
    T_i = mass*(u_i + g)/num_motors;
        if T_i >= thrust_limit
            dxdt(2,1) = (num_motors*thrust_limit/mass) - g;
        elseif T_i <= 0
            dxdt(2,1) = -g;
        else
            dxdt(2,1) = u_i;
        end
    end
end
end